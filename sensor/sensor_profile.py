# 设备状态枚举
# 该枚举类定义了设备的各种状态，用于表示设备在不同操作阶段的状态信息
from enum import Enum, IntEnum
from queue import Queue
import struct
import time
from typing import Callable, Dict, List, Optional

import bleak
from bleak import (
    BleakClient,
    BleakGATTCharacteristic,
)

import sensor
from sensor.gforce import GForce
from sensor.sensor_data import DataType, Sample, SensorData
import asyncio
import threading

from sensor.sensor_data_context import SensorProfileDataCtx
from sensor.sensor_device import BLEDevice, DeviceInfo, DeviceStateEx
from sensor.utils import start_loop, sync_timer, timer
from contextlib import suppress
from dataclasses import dataclass

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
OYM_CMD_NOTIFY_CHAR_UUID = "f000ffe1-0451-4000-b000-000000000000"
OYM_DATA_NOTIFY_CHAR_UUID = "f000ffe2-0451-4000-b000-000000000000"

RFSTAR_SERVICE_GUID = "00001812-0000-1000-8000-00805f9b34fb"
RFSTAR_CMD_UUID = "00000002-0000-1000-8000-00805f9b34fb"
RFSTAR_DATA_UUID = "00000003-0000-1000-8000-00805f9b34fb"


class SensorProfile:
    """
    SensorProfile 类用于蓝牙设备的连接，获取详细设备信息，初始化，数据接收。

    包含回调函数，用于处理传感器的状态变化、错误、数据接收和电量变化等事件。
    """

    def __init__(
        self,
        device: bleak.BLEDevice,
        adv: bleak.AdvertisementData,
        gforce_event_loop: asyncio.AbstractEventLoop,
    ):
        """
        初始化 SensorProfile 类的实例。

        :param            device (BLEDevice): 蓝牙设备对象，包含设备的名称、地址和信号强度等信息。
        """
        self._detail_device = device
        self._device = BLEDevice(device.name, device.address, adv.rssi)
        self._device_state = DeviceStateEx.Disconnected
        self._on_state_changed: Callable[["SensorProfile", DeviceStateEx], None] = None
        self._on_error_callback: Callable[["SensorProfile", str], None] = None
        self._on_data_callback: Callable[["SensorProfile", SensorData], None] = None
        self._on_power_changed: Callable[["SensorProfile", int], None] = None
        self._power = -1
        self._power_interval = 0
        self._adv = adv
        self._data_ctx: SensorProfileDataCtx = None
        self._gforce: GForce = None
        self._data_event_loop: asyncio.AbstractEventLoop = None
        self._gforce_event_loop: asyncio.AbstractEventLoop = gforce_event_loop
        self._event_loop: asyncio.AbstractEventLoop = None
        self._event_thread = None

    def __del__(self) -> None:
        """
        反初始化 SensorProfile 类的实例。

        """
        self._destroy()

    def _destroy(self):
        if (
            self._device_state == DeviceStateEx.Connected
            or self._device_state == DeviceStateEx.Ready
        ):
            self.disconnect()
        if self._data_event_loop != None:
            try:
                self._data_event_loop.stop()
                self._data_event_loop.close()
                self._data_event_thread.join()
            except Exception as e:
                pass
        if self._event_loop != None:
            try:
                self._event_loop.stop()
                self._event_loop.close()
                self._event_thread.join()
            except Exception as e:
                pass

    @property
    def deviceState(self) -> DeviceStateEx:
        """
        获取蓝牙连接状态。

        :return:            DeviceStateEx: 设备的状态，如 Disconnected、Connecting、Connected 等。
        """
        return self._device_state

    def _set_device_state(self, newState: DeviceStateEx):
        if self._device_state != newState:
            self._device_state = newState
            if self._on_state_changed != None:
                try:
                    self._event_loop.call_soon_threadsafe(
                        self._on_state_changed, self, newState
                    )
                except Exception as e:
                    print(e)
                    pass

    @property
    def hasInited(self) -> bool:
        """
        检查传感器是否已经初始化。

        :return:            bool: 如果传感器已经初始化，返回 True；否则返回 False。
        """
        if self._data_ctx == None:
            return False
        return self._data_ctx.hasInit()

    @property
    def isDataTransfering(self) -> bool:
        """
        检查传感器是否正在进行数据传输。

        :return:            bool: 如果传感器正在进行数据传输，返回 True；否则返回 False。
        """
        if self._data_ctx == None:
            return False
        return self._data_ctx.isDataTransfering

    @property
    def BLEDevice(self) -> BLEDevice:
        """
        获取传感器的蓝牙设备信息。

        :return:            BLEDevice: 蓝牙设备对象，包含设备的名称、地址和信号强度等信息。
        """
        return self._device

    @property
    def onStateChanged(self) -> Callable[["SensorProfile", DeviceStateEx], None]:
        """
        获取状态变化的回调函数。

        :return:            Callable[['SensorProfile', DeviceStateEx], None]: 状态变化的回调函数。
        """
        return self._on_state_changed

    @onStateChanged.setter
    def onStateChanged(
        self, callback: Callable[["SensorProfile", DeviceStateEx], None]
    ):
        """
        设置状态变化的回调函数。

        :param            callback (Callable[['SensorProfile', DeviceStateEx], None]): 状态变化的回调函数。
        """
        self._on_state_changed = callback

    @property
    def onErrorCallback(self) -> Callable[["SensorProfile", str], None]:
        """
        获取错误回调函数。

        :return:            Callable[['SensorProfile', str], None]: 错误回调函数。
        """
        return self._on_error_callback

    @onErrorCallback.setter
    def onErrorCallback(self, callback: Callable[["SensorProfile", str], None]):
        """
        设置错误回调函数。

        :param            callback (Callable[['SensorProfile', str], None]): 错误回调函数。
        """
        self._on_error_callback = callback

    @property
    def onDataCallback(self) -> Callable[["SensorProfile", SensorData], None]:
        """
        获取数据接收的回调函数。

        :return:            Callable[['SensorProfile', SensorData], None]: 数据接收的回调函数。
        """
        return self._on_data_callback

    @onDataCallback.setter
    def onDataCallback(self, callback: Callable[["SensorProfile", SensorData], None]):
        """
        设置数据接收的回调函数。

        :param            callback (Callable[['SensorProfile', SensorData], None]): 数据接收的回调函数。
        """
        self._on_data_callback = callback

    @property
    def onPowerChanged(self) -> Callable[["SensorProfile", int], None]:
        """
        获取电量变化的回调函数。

        :return:            Callable[['SensorProfile', int], None]: 电量变化的回调函数。
        """
        return self._on_power_changed

    @onPowerChanged.setter
    def onPowerChanged(self, callback: Callable[["SensorProfile", int], None]):
        """
        设置电量变化的回调函数。

        :param            callback (Callable[['SensorProfile', int], None]): 电量变化的回调函数。
        """
        self._on_power_changed = callback

    async def _connect(self) -> bool:
        if (
            self.deviceState == DeviceStateEx.Connected
            or self.deviceState == DeviceStateEx.Ready
        ):
            return True
        self._set_device_state(DeviceStateEx.Connecting)

        def handle_disconnect(_: BleakClient):
            self.stopDataNotification()
            self._data_ctx.close()
            time.sleep(1)
            self._data_buffer.queue.clear()
            self._data_ctx = None
            self._gforce = None
            self._set_device_state(DeviceStateEx.Disconnected)
            pass

        await self._gforce.connect(handle_disconnect, self._raw_data_buf)

        if self._gforce.client.is_connected:
            self._set_device_state(DeviceStateEx.Connected)
            if self._gforce.client.mtu_size >= 80:
                self._set_device_state(DeviceStateEx.Ready)
            else:
                self.disconnect()
        else:
            self._set_device_state(DeviceStateEx.Disconnected)

        return True

    def connect(self) -> bool:
        """
        连接传感器。

        :return:            bool: 如果连接成功，返回 True；否则返回 False。

        """
        if self._event_thread == None:
            self._event_loop = asyncio.new_event_loop()
            self._event_thread = threading.Thread(
                target=start_loop, args=(self._event_loop,)
            )
            self._event_thread.daemon = True
            self._event_thread.name = self._device.Name + " event"
            self._event_thread.start()
            self._data_buffer: Queue[SensorData] = Queue()
            self._raw_data_buf: Queue[bytes] = Queue()

        if self._gforce == None:
            if self._adv.service_data.get(SERVICE_GUID) != None:
                # print("OYM_SERVICE:" + self._detail_device.name)
                self._gforce = GForce(
                    self._detail_device,
                    OYM_CMD_NOTIFY_CHAR_UUID,
                    OYM_DATA_NOTIFY_CHAR_UUID,
                    False,
                )
            elif self._adv.service_data.get(RFSTAR_SERVICE_GUID) != None:
                # print("RFSTAR_SERVICE:" + self._detail_device.name)
                self._gforce = GForce(
                    self._detail_device, RFSTAR_CMD_UUID, RFSTAR_DATA_UUID, True
                )
                self._data_event_loop = asyncio.new_event_loop()
                self._data_event_thread = threading.Thread(
                    target=start_loop, args=(self._data_event_loop,)
                )
                self._data_event_thread.daemon = True
                self._data_event_thread.name = self._detail_device.name + " data"
                self._data_event_thread.start()
            else:
                print(
                    "Invalid device service uuid:"
                    + self._detail_device.name
                    + str(self._adv)
                )
                return False

        if self._data_ctx == None and self._gforce != None:
            self._data_ctx = SensorProfileDataCtx(
                self._gforce, self._device.Address, self._raw_data_buf
            )
            if self._data_ctx.isUniversalStream:
                timer(self._data_event_loop, 0, self._process_universal_data())

        result = sync_timer(self._gforce_event_loop, 0, self._connect())
        return result

    async def _waitForDisconnect(self) -> bool:
        while self.deviceState != DeviceStateEx.Disconnected:
            asyncio.sleep(1)
        return True

    async def _disconnect(self) -> bool:
        if (
            self.deviceState != DeviceStateEx.Connected
            and self.deviceState != DeviceStateEx.Ready
        ):
            return True
        if self._data_ctx == None:
            return False
        self._set_device_state(DeviceStateEx.Disconnecting)
        await self._gforce.disconnect()
        await asyncio.wait_for(self._waitForDisconnect(), 5)
        return True

    def disconnect(self) -> bool:
        """
        断开传感器连接。

        :return:            bool: 如果断开连接成功，返回 True;否则返回 False。

        """
        return sync_timer(self._gforce_event_loop, 0, self._disconnect())

    async def _process_data(self):
        while self._data_ctx._is_running and self._data_ctx.isDataTransfering:
            self._data_ctx.process_data(self._data_buffer)
            while self._data_ctx._is_running and self._data_ctx.isDataTransfering:
                sensorData: SensorData = None
                try:
                    sensorData = self._data_buffer.get_nowait()
                except Exception as e:
                    break
                if sensorData != None and self._on_data_callback != None:
                    try:
                        self._event_loop.call_soon_threadsafe(
                            self._on_data_callback, self, sensorData
                        )
                    except Exception as e:
                        print(e)
                self._data_buffer.task_done()

    async def _process_universal_data(self):
        self._data_ctx.processUniversalData(
            self._data_buffer, self._event_loop, self, self._on_data_callback
        )

    async def _startDataNotification(self) -> bool:
        result = await self._data_ctx.start_streaming()
        self._data_buffer.queue.clear()
        self._data_ctx.clear()
        if not self._data_ctx.isUniversalStream:
            timer(self._data_event_loop, 0, self._process_data())
        return result

    def startDataNotification(self) -> bool:
        """
        开始数据通知。

        :return:            bool: 如果开始数据通知成功，返回 True；否则返回 False。

        """
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if self._data_ctx == None:
            return False
        if not self._data_ctx.hasInit():
            return False

        if self._data_ctx.isDataTransfering:
            return True

        if self._data_event_loop == None:
            self._data_event_loop = asyncio.new_event_loop()
            self._data_event_thread = threading.Thread(
                target=start_loop, args=(self._data_event_loop,)
            )
            self._data_event_thread.daemon = True
            self._data_event_thread.name = self.BLEDevice.Name + " data"
            self._data_event_thread.start()

        return sync_timer(self._gforce_event_loop, 0, self._startDataNotification())

    async def _stopDataNotification(self) -> bool:

        return not await self._data_ctx.stop_streaming()

    def stopDataNotification(self) -> bool:
        """
        停止数据通知。

        :return:            bool: 如果停止数据通知成功，返回 True；否则返回 False。

        """
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if self._data_ctx == None:
            return False
        if not self._data_ctx.hasInit():
            return False

        if not self._data_ctx.isDataTransfering:
            return True

        return sync_timer(self._gforce_event_loop, 0, self._stopDataNotification())

    async def _refresh_power(self):
        self._power = await self._gforce.get_battery_level()

        if self._on_power_changed != None:
            try:
                self._event_loop.call_soon_threadsafe(
                    self._on_power_changed, self, self._power
                )
            except Exception as e:
                print(e)

        if self.deviceState == DeviceStateEx.Ready:
            timer(
                self._gforce_event_loop,
                self._power_interval / 1000,
                self._refresh_power(),
            )

    async def _init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if self._data_ctx == None:
            return False
        if self._data_ctx.hasInit():
            return True

        if await self._data_ctx.init(packageSampleCount):
            self._power_interval = powerRefreshInterval
            timer(
                self._gforce_event_loop,
                self._power_interval / 1000,
                self._refresh_power(),
            )

        return self._data_ctx.hasInit()

    def init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        """
        初始化数据采集。

        :param    packageSampleCount (int): 数据包中的样本数量。
        :param    powerRefreshInterval (int): 电量刷新间隔。

        :return:            bool: 初始化结果。True 表示成功，False 表示失败。

        """
        return sync_timer(
            self._gforce_event_loop,
            0,
            self._init(packageSampleCount, powerRefreshInterval),
        )

    def getBatteryLevel(self) -> int:
        """
        获取传感器的电池电量。

        :return:            int: 传感器的电池电量。 正常0-100，-1为未知。

        """
        return self._power

    def getDeviceInfo(self) -> Optional[DeviceInfo]:
        """
        获取传感器的设备信息。

        :return:            DeviceInfo: 传感器的设备信息。

        """
        if self.hasInited:
            return self._data_ctx._device_info
        return None

    def setParam(self, key: str, value: str) -> str:
        """
        设置传感器的参数。

        :param    key (str): 参数的键。
        :param    value (str): 参数的值。

        :return:            str: 设置参数的结果。

        """
        return ""
