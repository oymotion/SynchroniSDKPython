

from enum import Enum, IntEnum
from queue import Queue
import threading
import time
from typing import Callable, Optional
import uuid

import asyncio
import sys

from sensor import sensor_utils
from sensor.sensor_data import SensorData

from sensor.sensor_device import BLEDevice, DeviceInfo, DeviceStateEx
from sensor.sensor_utils import async_call, sync_call, async_exec

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
RFSTAR_SERVICE_GUID = "00001812-0000-1000-8000-00805f9b34fb"

_STATE_NAME_MAP = {
    "Disconnected": DeviceStateEx.Disconnected,
    "Connecting": DeviceStateEx.Connecting,
    "Connected": DeviceStateEx.Connected,
    "Ready": DeviceStateEx.Ready,
    "Disconnecting": DeviceStateEx.Disconnecting,
}


class SensorProfile:


    def __init__(
        self,
        device=None,
        adv=None,
        mac=None,
        serialized=None,
        bleak_host=None,
    ):

        self._bleak_host = bleak_host

        if serialized is not None:
            self._device_address = serialized["address"]
            self._device_name = serialized["name"]
            self._device_mac = serialized["mac"]
            self._device_rssi = serialized["rssi"]
            self._service_data = {
                k: bytes.fromhex(v) if isinstance(v, str) else v
                for k, v in serialized["service_data"].items()
            }
        else:
            if device is not None:
                if hasattr(device, "Name"):
                    # Our BLEDevice wrapper
                    self._device_name = device.Name
                    self._device_mac = device.Address if hasattr(device, "Address") else mac
                    self._device_rssi = device.RSSI if hasattr(device, "RSSI") else 0
                else:
                    # bleak BLEDevice
                    self._device_name = device.name
                    self._device_mac = mac
                    self._device_rssi = adv.rssi if adv else 0
                self._device_address = getattr(device, "address", self._device_mac)
            else:
                self._device_name = ""
                self._device_mac = mac
                self._device_rssi = 0
                self._device_address = mac

            self._service_data = {}
            if adv is not None and hasattr(adv, "service_data"):
                self._service_data = adv.service_data

        self._device = BLEDevice(self._device_name, self._device_mac, self._device_rssi)
        self._device_state = DeviceStateEx.Disconnected
        self._on_state_changed: Callable[["SensorProfile", DeviceStateEx], None] = None
        self._on_error_callback: Callable[["SensorProfile", str], None] = None
        self._on_data_callback: Callable[["SensorProfile", SensorData], None] = None
        self._on_power_changed: Callable[["SensorProfile", int], None] = None
        self._power = -1
        self._power_interval = 0
        self._is_starting = False
        self._is_setting_param = False
        self._has_inited = False
        self._is_data_transfering = False
        self._device_info: Optional[DeviceInfo] = None

    def __del__(self) -> None:

        if not sys.is_finalizing():
            self._destroy()

    def _destroy(self):
        try:
            if self._device_state == DeviceStateEx.Connected or self._device_state == DeviceStateEx.Ready:
                self.disconnect()
        except Exception:
            pass
        self._is_starting = False
        self._is_setting_param = False

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def _send_cmd_sync(self, cmd: dict, timeout: float = 10.0) -> dict:

        if self._bleak_host is None:
            return {}
        cmd["device_mac"] = self._device.Address
        return self._bleak_host.send_command_sync(cmd, timeout=timeout)

    async def _send_cmd_async(self, cmd: dict, timeout: float = 10.0) -> dict:

        if self._bleak_host is None:
            return {}
        cmd["device_mac"] = self._device.Address
        return await self._bleak_host.send_command_async(cmd, timeout=timeout)

    def _on_subprocess_message(self, msg: dict):

        msg_type = msg.get("type")
        if msg_type == "state_changed":
            state_name = msg.get("state")
            new_state = _STATE_NAME_MAP.get(state_name, self._device_state)
            self._set_device_state(new_state)
        elif msg_type == "power_changed":
            power = msg.get("power", -1)
            if power < self._power or self._power == -1:
                self._power = power
            if self._on_power_changed is not None:
                try:
                    self._on_power_changed(self, self._power)
                except Exception:
                    pass
        elif msg_type == "sensor_data":
            if self._on_data_callback is not None:
                try:
                    self._on_data_callback(self, msg.get("data"))
                except Exception:
                    pass
        elif msg_type == "error":
            if self._on_error_callback is not None:
                try:
                    self._on_error_callback(self, msg.get("message", ""))
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def deviceState(self) -> DeviceStateEx:

        return self._device_state

    def _set_device_state(self, newState: DeviceStateEx):
        if self._device_state != newState:
            self._device_state = newState
            if newState == DeviceStateEx.Disconnected:
                self._has_inited = False
                self._is_data_transfering = False
            if self._on_state_changed is not None:
                try:
                    self._on_state_changed(self, newState)
                except Exception as e:
                    raise RuntimeError("Set device state %s fail: %s" % (self.BLEDevice.Name , e))

    @property
    def hasInited(self) -> bool:

        return self._has_inited

    @property
    def isDataTransfering(self) -> bool:

        return self._is_data_transfering

    @property
    def BLEDevice(self) -> BLEDevice:

        return self._device

    @property
    def onStateChanged(self) -> Callable[["SensorProfile", DeviceStateEx], None]:

        return self._on_state_changed

    @onStateChanged.setter
    def onStateChanged(self, callback: Callable[["SensorProfile", DeviceStateEx], None]):

        self._on_state_changed = callback

    @property
    def onErrorCallback(self) -> Callable[["SensorProfile", str], None]:

        return self._on_error_callback

    @onErrorCallback.setter
    def onErrorCallback(self, callback: Callable[["SensorProfile", str], None]):

        self._on_error_callback = callback

    @property
    def onDataCallback(self) -> Callable[["SensorProfile", SensorData], None]:

        return self._on_data_callback

    @onDataCallback.setter
    def onDataCallback(self, callback: Callable[["SensorProfile", SensorData], None]):

        self._on_data_callback = callback

    @property
    def onPowerChanged(self) -> Callable[["SensorProfile", int], None]:

        return self._on_power_changed

    @onPowerChanged.setter
    def onPowerChanged(self, callback: Callable[["SensorProfile", int], None]):

        self._on_power_changed = callback

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    async def _connect(self) -> bool:
        from sensor import sensor_utils
        if sensor_utils._terminated:
            return False
        if self.deviceState == DeviceStateEx.Connected or self.deviceState == DeviceStateEx.Ready:
            return True

        self._set_device_state(DeviceStateEx.Connecting)

        cmd = {
            "type": "connect",
            "device_address": self._device_address,
            "name": self._device_name,
            "service_data": self._service_data,
        }
        result = await self._send_cmd_async(cmd, timeout=sensor_utils._TIMEOUT)
        success = result.get("success", False) if result else False
        return success

    def connect(self) -> bool:

        result = sync_call(self._connect())
        return result

    async def asyncConnect(self) -> bool:

        return await async_call(self._connect())

    async def _disconnect(self) -> bool:
        if self.deviceState != DeviceStateEx.Connected and self.deviceState != DeviceStateEx.Ready:
            return True

        cmd = {"type": "disconnect"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        self._has_inited = False
        self._is_data_transfering = False
        return True

    def disconnect(self) -> bool:

        return sync_call(self._disconnect())

    async def asyncDisconnect(self) -> bool:

        return await async_call(self._disconnect())

    # ------------------------------------------------------------------
    # Data notification
    # ------------------------------------------------------------------
    async def _startDataNotification(self) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if not self._has_inited:
            return False

        cmd = {"type": "start_notification"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        success = result.get("success", False) if result else False
        if success:
            self._is_data_transfering = True
        return success

    def startDataNotification(self) -> bool:

        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = sync_call(self._startDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            raise(e)

    async def asyncStartDataNotification(self) -> bool:

        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = await async_call(self._startDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            raise(e)

    async def _stopDataNotification(self) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if not self._has_inited:
            return False

        cmd = {"type": "stop_notification"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        success = result.get("success", False) if result else False
        if success:
            self._is_data_transfering = False
        return success

    def stopDataNotification(self) -> bool:

        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = sync_call(self._stopDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            raise(e)

    async def asyncStopDataNotification(self) -> bool:

        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = await async_call(self._stopDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            raise(e)

    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------
    async def _init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False

        self._power_interval = powerRefreshInterval

        cmd = {
            "type": "init",
            "package_sample_count": packageSampleCount,
            "power_refresh_interval": powerRefreshInterval,
        }
        result = await self._send_cmd_async(cmd, timeout=20.0)
        success = result.get("success", False) if result else False
        if success:
            self._has_inited = True
            self._device_info = result.get("device_info")
        return success

    def init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:

        return sync_call(
            self._init(packageSampleCount, powerRefreshInterval),
            20,
        )

    async def asyncInit(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:

        return await async_call(
            self._init(packageSampleCount, powerRefreshInterval),
            20,
        )

    # ------------------------------------------------------------------
    # Battery
    # ------------------------------------------------------------------
    async def _asyncGetBatteryLevel(self) -> int:
        if self.deviceState != DeviceStateEx.Ready:
            return -1
        if not self._has_inited:
            return -1

        cmd = {"type": "get_battery"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", -1)
        return -1

    async def asyncGetBatteryLevel(self) -> int:

        return await async_call(self._asyncGetBatteryLevel())

    def getBatteryLevel(self) -> int:

        return self._power

    def getDeviceInfo(self) -> Optional[DeviceInfo]:

        if self.hasInited:
            return self._device_info
        return None

    # ------------------------------------------------------------------
    # Neucir
    # ------------------------------------------------------------------
    async def _asyncSet_neucir_app_control(self, open: bool, close: bool, stop: bool) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"
        if not self._has_inited:
            return "Error: Not initialized"

        cmd = {
            "type": "set_neucir_app_control",
            "open": open,
            "close": close,
            "stop": stop,
        }
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", "OK")
        return result.get("result", "Error: Unknown error") if result else "Error: Unknown error"

    async def _asyncSet_neucir_mode(self, mode: int) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"
        if not self._has_inited:
            return "Error: Not initialized"

        cmd = {"type": "set_neucir_mode", "mode": mode}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", "OK")
        return result.get("result", "Error: Unknown error") if result else "Error: Unknown error"

    # ------------------------------------------------------------------
    # SetParam
    # ------------------------------------------------------------------
    async def _setParam(self, key: str, value: str) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"

        cmd = {"type": "set_param", "key": key, "value": value}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result:
            return result.get("result", "Error: Unknown error")
        return "Error: Timeout"

    def setParam(self, key: str, value: str) -> str:

        if self._is_setting_param:
            return "Error: Please wait for the previous operation to complete"

        try:
            self._is_setting_param = True
            ret = sync_call(
                self._setParam(key, value),
                10,
            )
            self._is_setting_param = False
            return ret
        except Exception as e:
            self._is_setting_param = False
            raise(e)

    async def asyncSetParam(self, key: str, value: str) -> str:

        if self._is_setting_param:
            return "Error: Please wait for the previous operation to complete"

        try:
            self._is_setting_param = True
            ret = await async_call(
                self._setParam(key, value),
                10,
            )
            self._is_setting_param = False
            return ret
        except Exception as e:
            self._is_setting_param = False
            raise(e)
