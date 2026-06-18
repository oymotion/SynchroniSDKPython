import asyncio
import multiprocessing
import platform
import queue
import threading
import time

import bleak
from bleak import BleakScanner
from bleak import AdvertisementData

from sensor import sensor_utils

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
RFSTAR_SERVICE_GUID = "00001812-0000-1000-8000-00805f9b34fb"

OYM_CMD_NOTIFY_CHAR_UUID = "f000ffe1-0451-4000-b000-000000000000"
OYM_DATA_NOTIFY_CHAR_UUID = "f000ffe2-0451-4000-b000-000000000000"

RFSTAR_CMD_UUID = "00000002-0000-1000-8000-00805f9b34fb"
RFSTAR_DATA_UUID = "00000003-0000-1000-8000-00805f9b34fb"


def _extract_mac(_device: bleak.BLEDevice, adv: AdvertisementData) -> str:

    mac = None
    if adv.service_data.get(SERVICE_GUID) is not None:
        bytes_val = adv.service_data[SERVICE_GUID]
        mac = ":".join(f"{byte:02X}" for byte in bytes_val)
    elif adv.service_data.get(RFSTAR_SERVICE_GUID) is not None:
        bytes_val = adv.service_data[RFSTAR_SERVICE_GUID]
        mac = ":".join(f"{byte:02X}" for byte in reversed(bytes_val))
    return mac


def _serialize_device(_device: bleak.BLEDevice, adv: AdvertisementData) -> dict:

    mac = _extract_mac(_device, adv)
    if mac is None:
        return None
    return {
        "address": _device.address,
        "name": _device.name,
        "rssi": adv.rssi,
        "mac": mac,
        "service_uuids": list(adv.service_uuids),
        "service_data": {
            k: v.hex() if isinstance(v, bytes) else v
            for k, v in adv.service_data.items()
        },
    }


def _match_device(_device: bleak.BLEDevice, adv: AdvertisementData):

    if _device.name is None:
        return False
    if SERVICE_GUID in adv.service_uuids:
        return True
    return False


class BleakProcess(multiprocessing.Process):


    def __init__(
        self,
        cmd_queue: multiprocessing.Queue,
        result_queue: multiprocessing.Queue,
    ):
        super().__init__(daemon=True)
        self.cmd_queue = cmd_queue
        self.result_queue = result_queue
        self._scanner = None
        self._is_scanning = False
        self._should_exit = False

        # Per-device state (all in sub-process)
        self._devices: dict = {}          # mac -> bleak.BLEDevice
        self._gforces: dict = {}          # mac -> GForce
        self._data_ctxs: dict = {}        # mac -> SensorProfileDataCtx
        self._raw_bufs: dict = {}         # mac -> queue.Queue
        self._device_states: dict = {}    # mac -> str state name
        self._power_intervals: dict = {}  # mac -> int (ms)
        self._data_tasks: dict = {}       # mac -> asyncio.Task
        self._battery_tasks: dict = {}    # mac -> asyncio.Task

        # Per-device event loops and threads
        self._event_loops: dict = {}      # mac -> asyncio.AbstractEventLoop
        self._event_threads: dict = {}    # mac -> threading.Thread
        self._data_event_loops: dict = {}    # mac -> asyncio.AbstractEventLoop
        self._data_event_threads: dict = {}  # mac -> threading.Thread
        self._cleanup_locks: dict = {}    # mac -> asyncio.Lock

        # Singleton gforce loop for all bleak object creation/access
        self._gforce_event_loop = None
        self._gforce_event_thread = None

    def _publish(self, msg_type: str, **kwargs):

        try:
            msg = {"type": msg_type, **kwargs}

            if msg_type in ("sensor_data", "devices", "scan_once_result", "error"):
                if self._msg_queue.qsize() > sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE:
                    return
            self._msg_queue.put_nowait(msg)
        except Exception:
            pass

    def _flush_msg_queue(self):

        while True:
            try:
                msg = self._msg_queue.get_nowait()
                msg_type = msg.get("type")
                if msg_type in ("sensor_data", "devices", "scan_once_result", "error"):
                    self.result_queue.put_nowait(msg)
                else:
                    self.result_queue.put(msg, timeout=sensor_utils._TIMEOUT)
            except queue.Empty:
                break
            except queue.Full:
                break
            except Exception:
                break

    async def _publisher_task(self):

        while True:
            try:
                msg = self._msg_queue.get_nowait()
                msg_type = msg.get("type")
                if msg_type in ("sensor_data", "devices", "scan_once_result", "error"):
                    self.result_queue.put_nowait(msg)
                else:
                    self.result_queue.put(msg, timeout=sensor_utils._TIMEOUT)
            except queue.Empty:
                if self._should_exit:
                    break
                await asyncio.sleep(0.001)
            except queue.Full:
                pass
            except Exception:
                pass

    def _init_scanner(self):
        if self._scanner is None:
            self._scanner = BleakScanner(
                detection_callback=_match_device,
                service_uuids=[RFSTAR_SERVICE_GUID, SERVICE_GUID],
            )

    def _ensure_gforce_loop(self):
        from sensor import sensor_utils
        if self._gforce_event_loop is None or self._gforce_event_loop.is_closed():
            self._gforce_event_loop = asyncio.new_event_loop()
            self._gforce_event_thread = threading.Thread(
                target=sensor_utils.start_loop, args=(self._gforce_event_loop,)
            )
            self._gforce_event_thread.daemon = True
            self._gforce_event_thread.name = "gforce_event"
            self._gforce_event_thread.start()
        return self._gforce_event_loop

    def _ensure_device_loops(self, device_mac: str):
        from sensor import sensor_utils

        event_loop = self._event_loops.get(device_mac)
        data_event_loop = self._data_event_loops.get(device_mac)
        if (event_loop is not None and not event_loop.is_closed() and
                data_event_loop is not None and not data_event_loop.is_closed()):
            self._cleanup_locks.setdefault(device_mac, asyncio.Lock())
            return

        event_loop = asyncio.new_event_loop()
        event_thread = threading.Thread(target=sensor_utils.start_loop, args=(event_loop,))
        event_thread.daemon = True
        event_thread.name = device_mac + "_event"
        event_thread.start()

        gforce_event_loop = self._ensure_gforce_loop()

        data_event_loop = asyncio.new_event_loop()
        data_event_thread = threading.Thread(target=sensor_utils.start_loop, args=(data_event_loop,))
        data_event_thread.daemon = True
        data_event_thread.name = device_mac + "_data_event"
        data_event_thread.start()

        self._event_loops[device_mac] = event_loop
        self._event_threads[device_mac] = event_thread
        self._data_event_loops[device_mac] = data_event_loop
        self._data_event_threads[device_mac] = data_event_thread
        self._cleanup_locks[device_mac] = asyncio.Lock()

    def _stop_device_loops(self, device_mac: str):

        # Cancel data task first if still running in data_event_loop
        if device_mac in self._data_tasks:
            task = self._data_tasks.pop(device_mac, None)
            if task:
                try:
                    loop = self._data_event_loops.get(device_mac)
                    if loop and not loop.is_closed():
                        loop.call_soon_threadsafe(task.cancel)
                except Exception:
                    pass

        # Stop loops and join threads (gforce loop is singleton, do not stop here)
        for loop_dict, thread_dict in [
            (self._data_event_loops, self._data_event_threads),
            (self._event_loops, self._event_threads),
        ]:
            loop = loop_dict.pop(device_mac, None)
            thread = thread_dict.pop(device_mac, None)
            if loop is not None and not loop.is_closed():
                try:
                    loop.call_soon_threadsafe(loop.stop)
                except Exception:
                    pass
            if thread is not None and thread.is_alive():
                try:
                    thread.join(timeout=2)
                except Exception:
                    pass

        self._cleanup_locks.pop(device_mac, None)

    def run(self):

        if platform.system() == "Windows":
            try:
                from bleak.backends.winrt.util import allow_sta
                allow_sta()
            except ImportError:
                pass


        # "cannot pickle '_thread.lock' object"
        self._msg_queue = queue.Queue()

        async def _run_main():
            publisher_task = asyncio.create_task(self._publisher_task())
            try:
                await self._main_loop()
            finally:
                self._should_exit = True

                try:
                    await asyncio.wait_for(publisher_task, timeout=2.0)
                except asyncio.TimeoutError:
                    publisher_task.cancel()
                    try:
                        await publisher_task
                    except asyncio.CancelledError:
                        pass

                self._flush_msg_queue()

        asyncio.run(_run_main())

        # Stop singleton gforce loop
        if self._gforce_event_loop is not None and not self._gforce_event_loop.is_closed():
            try:
                self._gforce_event_loop.call_soon_threadsafe(self._gforce_event_loop.stop)
            except Exception:
                pass
        if self._gforce_event_thread is not None and self._gforce_event_thread.is_alive():
            try:
                self._gforce_event_thread.join(timeout=2)
            except Exception:
                pass

        for device_mac in list(self._event_loops.keys()):
            self._stop_device_loops(device_mac)

    async def _main_loop(self):

        while not self._should_exit:
            try:
                cmd = self.cmd_queue.get_nowait()
                asyncio.create_task(self._handle_command(cmd))
            except multiprocessing.queues.Empty:
                await asyncio.sleep(0.05)


        await self._cleanup_all_devices()

    async def _cleanup_all_devices(self):
        for device_mac in list(self._gforces.keys()):
            loop = self._event_loops.get(device_mac)
            if loop is not None and not loop.is_closed():
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        self._cleanup_device(device_mac, disconnect_client=False), loop
                    )
                    await asyncio.wait_for(asyncio.wrap_future(future), timeout=3)
                except Exception:
                    pass
            self._stop_device_loops(device_mac)

    async def _handle_command(self, cmd: dict):
        cmd_type = cmd.get("type")

        # Scan commands run in singleton gforce loop
        if cmd_type == "scan_once":
            loop = self._ensure_gforce_loop()
            asyncio.run_coroutine_threadsafe(
                self._do_scan_once(cmd.get("period", 5000)), loop
            )
            return
        if cmd_type == "start_scan":
            loop = self._ensure_gforce_loop()
            asyncio.run_coroutine_threadsafe(
                self._do_start_scan(cmd.get("period", 5000)), loop
            )
            return
        if cmd_type == "stop_scan":
            self._is_scanning = False
            return
        if cmd_type == "terminate":
            self._is_scanning = False
            self._should_exit = True
            return

        # Connect runs in main loop because it creates device loops
        if cmd_type == "connect":
            await self._do_connect(cmd)
            return

        # All other device commands run in the device's event_loop
        device_cmd_handlers = {
            "disconnect": self._do_disconnect,
            "init": self._do_init,
            "start_notification": self._do_start_notification,
            "stop_notification": self._do_stop_notification,
            "get_battery": self._do_get_battery,
            "set_neucir_app_control": self._do_set_neucir_app_control,
            "set_neucir_mode": self._do_set_neucir_mode,
            "set_param": self._do_set_param,
        }

        if cmd_type not in device_cmd_handlers:
            return

        device_mac = cmd.get("device_mac")
        loop = self._event_loops.get(device_mac)
        if loop is None or loop.is_closed():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Device not connected",
            )
            return

        handler = device_cmd_handlers[cmd_type]
        future = asyncio.run_coroutine_threadsafe(handler(cmd), loop)
        try:
            await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
        except asyncio.TimeoutError:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Timeout",
            )
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )

    # ------------------------------------------------------------------
    # Scan commands (existing)
    # ------------------------------------------------------------------
    async def _do_scan_once(self, period: int):

        try:
            self._init_scanner()
            found_devices = await self._scanner.discover(
                timeout=period / 1000, return_adv=True
            )
            devices = self._process_ble_devices(found_devices)
            self._publish("scan_once_result", devices=devices)
        except Exception as e:
            self._publish("error", message=f"scan_once failed: {e}")

    async def _do_start_scan(self, period: int):

        try:
            self._init_scanner()
            found_devices = await self._scanner.discover(
                timeout=period / 1000, return_adv=True
            )
            devices = self._process_ble_devices(found_devices)
            self._publish("devices", devices=devices)
        except Exception as e:
            self._publish("error", message=f"start_scan failed: {e}")

    def _process_ble_devices(self, found_devices: dict) -> list:

        devices = []
        for uuid in found_devices:
            device = found_devices[uuid][0]
            if device.name is None:
                continue
            adv = found_devices[uuid][1]
            if SERVICE_GUID in adv.service_uuids:
                serialized = _serialize_device(device, adv)
                if serialized is not None:
                    mac = serialized.get("mac")
                    if mac:
                        self._devices[mac] = device
                    devices.append(serialized)
        return devices

    # ------------------------------------------------------------------
    # Device connection commands
    # ------------------------------------------------------------------
    async def _do_connect(self, cmd: dict):

        device_mac = cmd["device_mac"]

        if device_mac in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=True,
            )
            return

        self._ensure_device_loops(device_mac)
        gforce_loop = self._ensure_gforce_loop()

        future = asyncio.run_coroutine_threadsafe(self._do_connect_inner(cmd), gforce_loop)
        try:
            await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
        except asyncio.TimeoutError:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Connect timeout",
            )
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )

    async def _do_connect_inner(self, cmd: dict):

        from sensor.gforce import GForce
        from sensor.sensor_data_context import SensorProfileDataCtx

        device_mac = cmd["device_mac"]
        device_address = cmd["device_address"]
        name = cmd.get("name", "")
        service_data = cmd.get("service_data", {})

        event_loop = self._event_loops.get(device_mac)
        gforce_event_loop = self._gforce_event_loop
        data_event_loop = self._data_event_loops.get(device_mac)

        if event_loop is None or gforce_event_loop is None or data_event_loop is None:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Event loops not initialized",
            )
            return

        # Determine service type
        if service_data.get(SERVICE_GUID) is not None:
            cmd_char = OYM_CMD_NOTIFY_CHAR_UUID
            data_char = OYM_DATA_NOTIFY_CHAR_UUID
            is_universal = False
        elif service_data.get(RFSTAR_SERVICE_GUID) is not None:
            cmd_char = RFSTAR_CMD_UUID
            data_char = RFSTAR_DATA_UUID
            is_universal = True
        else:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Invalid device service uuid",
            )
            return

        # Read bleak BLEDevice from scanned devices
        bleak_device = self._devices.get(device_mac)
        if bleak_device is None:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Device not found in scanned devices",
            )
            return

        # Create raw data buffer (local to sub-process)
        raw_buf = queue.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)

        # Create GForce with per-device event loops
        gforce = GForce(bleak_device, cmd_char, data_char, is_universal, event_loop, gforce_event_loop)

        # Define disconnect callback: schedule cleanup in event_loop
        def handle_disconnect(_):
            loop = self._event_loops.get(device_mac)
            if loop is not None and not loop.is_closed():
                try:
                    asyncio.run_coroutine_threadsafe(
                        self._cleanup_device(device_mac, disconnect_client=False), loop
                    )
                except Exception:
                    pass

        try:
            await gforce.connect(handle_disconnect, raw_buf)
        except Exception as e:
            await self._cleanup_device(device_mac, disconnect_client=False)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        if not gforce.client or not gforce.client.is_connected:
            await self._cleanup_device(device_mac, disconnect_client=False)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Connection failed",
            )
            return

        # Store state
        self._gforces[device_mac] = gforce
        self._raw_bufs[device_mac] = raw_buf
        self._device_states[device_mac] = "Connected"

        # Create data context
        data_ctx = SensorProfileDataCtx(gforce, device_mac, raw_buf)
        self._data_ctxs[device_mac] = data_ctx

        # Start data processing loop in data_event_loop
        asyncio.run_coroutine_threadsafe(self._device_data_loop(device_mac), data_event_loop)

        # Publish states
        self._publish("state_changed", device_mac=device_mac, state="Connected")
        self._publish("state_changed", device_mac=device_mac, state="Ready")
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=True,
        )

    async def _cleanup_device(self, device_mac: str, disconnect_client: bool = False):

        lock = self._cleanup_locks.get(device_mac)
        if lock is None:
            return

        async with lock:
            if device_mac not in self._gforces and not disconnect_client:
                return  # Already cleaned

            # Cancel data task (running in data_event_loop)
            if device_mac in self._data_tasks:
                task = self._data_tasks.pop(device_mac, None)
                if task is not None:
                    try:
                        task.cancel()
                    except Exception:
                        pass

            # Cancel battery task
            if device_mac in self._battery_tasks:
                self._battery_tasks.pop(device_mac, None)

            # Stop streaming and optionally disconnect client
            if disconnect_client and device_mac in self._gforces:
                if device_mac in self._data_ctxs:
                    ctx = self._data_ctxs[device_mac]
                    if ctx.isDataTransfering:
                        try:
                            await ctx.stop_streaming()
                        except Exception:
                            pass
                try:
                    await self._gforces[device_mac].disconnect()
                except Exception:
                    pass

            # Close data context
            if device_mac in self._data_ctxs:
                try:
                    self._data_ctxs[device_mac].close()
                except Exception:
                    pass
                self._data_ctxs.pop(device_mac, None)

            # Pop state
            self._gforces.pop(device_mac, None)
            self._raw_bufs.pop(device_mac, None)
            self._device_states[device_mac] = "Disconnected"

        self._publish("state_changed", device_mac=device_mac, state="Disconnected")

    async def _do_disconnect(self, cmd: dict):

        device_mac = cmd["device_mac"]
        state = self._device_states.get(device_mac, "Disconnected")

        if state not in ("Connected", "Ready"):
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=True,
            )
            return

        self._device_states[device_mac] = "Disconnecting"
        self._publish("state_changed", device_mac=device_mac, state="Disconnecting")

        await self._cleanup_device(device_mac, disconnect_client=True)

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=True,
        )

    async def _do_init(self, cmd: dict):

        device_mac = cmd["device_mac"]
        package_sample_count = cmd.get("package_sample_count", 16)
        power_refresh_interval = cmd.get("power_refresh_interval", 0)

        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not connected",
            )
            return

        ctx = self._data_ctxs[device_mac]

        try:
            success = await ctx.init(package_sample_count)
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        if success:
            self._power_intervals[device_mac] = power_refresh_interval

            # Get initial battery
            if device_mac in self._gforces:
                try:
                    power = await self._gforces[device_mac].get_battery_level()
                    self._publish("power_changed", device_mac=device_mac, power=power)
                except Exception:
                    pass

            # Start battery polling task (runs in event_loop)
            if power_refresh_interval > 0:
                self._battery_tasks[device_mac] = asyncio.create_task(
                    self._battery_loop(device_mac)
                )

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=success,
            result=success,
            device_info=ctx._device_info if success else None,
        )

    async def _battery_loop(self, device_mac: str):

        interval = self._power_intervals.get(device_mac, 0)
        while interval > 0 and device_mac in self._gforces:
            await asyncio.sleep(interval / 1000)
            try:
                power = await self._gforces[device_mac].get_battery_level()
                self._publish("power_changed", device_mac=device_mac, power=power)
            except Exception:
                break

    async def _device_data_loop(self, device_mac: str):

        from sensor.sensor_data import SensorData

        ctx = self._data_ctxs.get(device_mac)
        if ctx is None:
            return

        local_buf = queue.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)

        def on_data(sensor_data: SensorData):
            self._publish("sensor_data", device_mac=device_mac, data=sensor_data)

        def on_error(message: str):
            self._publish("error", device_mac=device_mac, message=message)

        try:
            if ctx.isUniversalStream:
                await ctx.processUniversalData(local_buf, on_data, on_error)
            else:
                await ctx.process_data(local_buf, on_data, on_error)
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

    async def _do_start_notification(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not initialized",
            )
            return

        ctx = self._data_ctxs[device_mac]
        if not ctx.hasInit():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not initialized",
            )
            return

        try:
            result = await ctx.start_streaming()
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_stop_notification(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not initialized",
            )
            return

        ctx = self._data_ctxs[device_mac]
        try:
            result = await ctx.stop_streaming()
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_get_battery(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=-1,
            )
            return

        try:
            power = await self._gforces[device_mac].get_battery_level()
        except Exception:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=-1,
            )
            return

        self._publish("power_changed", device_mac=device_mac, power=power)
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=power,
        )

    async def _do_set_neucir_app_control(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not connected",
            )
            return

        try:
            ret = await self._gforces[device_mac].set_neucir_app_control(
                cmd.get("open", False),
                cmd.get("close", False),
                cmd.get("stop", False),
            )
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        result = "OK" if ret else "Error: Unknown error"
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_set_neucir_mode(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not connected",
            )
            return

        try:
            ret = await self._gforces[device_mac].set_neucir_mode(cmd.get("mode", 0))
        except Exception as e:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        result = "OK" if ret else "Error: Unknown error"
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_set_param(self, cmd: dict):

        device_mac = cmd["device_mac"]
        key = cmd.get("key", "")
        value = cmd.get("value", "")

        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Error: Please connect first",
            )
            return

        ctx = self._data_ctxs[device_mac]
        if not ctx.hasInit():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Error: Not initialized",
            )
            return

        result = "Error: Not supported"

        if key in ["NTF_EMG", "NTF_EEG", "NTF_ECG", "NTF_IMU", "NTF_BRTH", "NTF_IMPEDANCE"]:
            if value in ["ON", "OFF"]:
                ctx.init_map[key] = value
                result = "OK"

        if key in ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]:
            if value in ["ON", "OFF"]:
                try:
                    result = await ctx.setFilter(key, value)
                except Exception as e:
                    result = "ERROR: " + str(e)

        if key == "DEBUG_BLE_DATA_PATH":
            try:
                result = await ctx.setDebugCSV(value)
            except Exception as e:
                result = "ERROR: " + str(e)

        if key == "NEUCIR_SET_MODE":
            if value in ["APP_REMOTE"]:
                try:
                    ret = await self._gforces[device_mac].set_neucir_mode(1)
                    result = "OK" if ret else "Error: Unknown error"
                except Exception as e:
                    result = "ERROR: " + str(e)

        if key == "NEUCIR_APP_CONTROL":
            if value in ["OPEN", "CLOSE", "STOP"]:
                try:
                    if value == "OPEN":
                        ret = await self._gforces[device_mac].set_neucir_app_control(True, False, False)
                    elif value == "CLOSE":
                        ret = await self._gforces[device_mac].set_neucir_app_control(False, True, False)
                    elif value == "STOP":
                        ret = await self._gforces[device_mac].set_neucir_app_control(False, False, True)
                    result = "OK" if ret else "Error: Unknown error"
                except Exception as e:
                    result = "ERROR: " + str(e)

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=(not result.startswith("Error")),
            result=result,
        )
