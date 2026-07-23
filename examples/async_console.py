import asyncio
import signal
import sys
import time
from typing import List
from sensor import *

SCAN_DEVICE_PERIOD_IN_MS = 3000
PACKAGE_COUNT = 5
POWER_REFRESH_PERIOD_IN_MS = 5000


def terminate():
    SensorControllerInstance.terminate()
    exit()


def run_bin_replay(bin_path: str) -> int:
    """离线回放 bin 文件，走完整解析链路做冒烟测试（不需要蓝牙硬件）。

    打印 bin 信息（config 版本、时长、设备），全速回放并统计收到的数据批次数；
    至少收到一批数据返回 0，否则返回 1。
    """
    controller = SensorControllerInstance
    info = controller.getBinFileInfo(bin_path)
    if info is None:
        print("invalid bin file: no config record found")
        return 1
    print(
        "bin info: config v%s, duration %.1fs, device %s (%s)"
        % (
            info.get("version"),
            info.get("replay_duration", 0.0),
            info.get("device_name"),
            info.get("device_mac"),
        )
    )
    sensor = controller.requireSensor(
        BLEDevice(info.get("device_name") or "", info.get("device_mac") or "", 0)
    )
    if sensor is None:
        print("failed to create SensorProfile for replay")
        return 1

    received = {"count": 0}

    def _on_data(sensor, data):
        received["count"] += 1

    sensor.onDataCallback = _on_data
    sensor.onErrorCallback = onErrorCallback

    profile = controller.replayBinFile(bin_path, sensor, realtime=False)
    controller.terminate()
    if profile is None:
        print("replay failed to start")
        return 1
    print("replay finished, received %d data batches" % received["count"])
    if received["count"] <= 0:
        print("no data parsed during replay")
        return 1
    return 0


async def main():
    signal.signal(signal.SIGINT, lambda signal, frame: terminate())

    # --replay <bin 文件>：离线回放冒烟测试（构建脚本用，无需蓝牙硬件）
    if len(sys.argv) > 1 and sys.argv[1] == "--replay":
        if len(sys.argv) < 3:
            print("usage: python async_console.py --replay <bin_file>")
            return 1
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, run_bin_replay, sys.argv[2])

    if not SensorControllerInstance.isEnable:
        print("please open bluetooth")
        return

    deviceList = await SensorControllerInstance.asyncScan(3000)

    filteredDevice = filter(
        lambda x: x.RSSI > -80 and (x.Name.startswith("OY")),
        deviceList,
    )
    for device in filteredDevice:
        sensor = SensorControllerInstance.requireSensor(device)
        if sensor == None:
            continue

        print("found: " + sensor.BLEDevice.Name)
        sensor.onDataCallback = onDataCallback
        sensor.onPowerChanged = onPowerChanged
        sensor.onStateChanged = onStateChanged
        sensor.onErrorCallback = onErrorCallback

        # check state & connect
        if sensor.deviceState != DeviceStateEx.Ready:
            print("connecting: " + sensor.BLEDevice.Address)
            if not await sensor.asyncConnect():
                print("connect device: " + sensor.BLEDevice.Name + " failed")
                continue

        # init & start data transfer
        if sensor.deviceState == DeviceStateEx.Ready and not sensor.hasInited:
            await sensor.asyncSetParam("DEBUG_BLE_DATA_PATH", "d:/temp/test.csv")
            # await sensor.asyncSetParam("NTF_ECG", "OFF")
            # await sensor.asyncSetParam("NTF_IMU", "OFF")
            # await sensor.asyncSetParam("FILTER_50HZ", "OFF")
            # await sensor.asyncSetParam("FILTER_60HZ", "OFF")
            # await sensor.asyncSetParam("FILTER_HPF", "OFF")
            # await sensor.asyncSetParam("FILTER_LPF", "OFF")
            if not await sensor.asyncInit(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
                print("init device: " + sensor.BLEDevice.Name + " failed")
                continue
            deviceInfo = sensor.getDeviceInfo()
            print("deviceInfo: Model: " + str(deviceInfo.ModelName))

        if sensor.hasInited:
            print("start data transfer")
            if not await sensor.asyncStartDataNotification():
                print("start data transfer with device: " + sensor.BLEDevice.Name + " failed")
                continue
    print("end")
    while True:
        await asyncio.sleep(10)
        for sensor in SensorControllerInstance.getConnectedSensors():
            if sensor.isDataTransfering:
                print("stop")
                await sensor.asyncStopDataNotification()
            else:
                print("start")
                await sensor.asyncStartDataNotification()

    # SensorControllerInstance.terminate()


def onDataCallback(sensor: SensorProfile, data: SensorData):
    # if (
    #     data.dataType == DataType.NTF_EEG
    #     or data.dataType == DataType.NTF_ECG
    #     or data.dataType == DataType.NTF_BRTH
    #     or data.dataType == DataType.NTF_ACC
    #     or data.dataType == DataType.NTF_GYRO
    # ):
    # print("got data from sensor: " + sensor.BLEDevice.Name + " data type: " + str(data.dataType))
    # print(str(data.channelSamples[0][0].sampleIndex))

    if data.channelSamples[0][0].sampleIndex == 1000:
        print("do stopDataNotification")
        sensor.stopDataNotification()

    if data.dataType == DataType.NTF_EMG:
        print(
            sensor.BLEDevice.Name
            + ":"
            + str(data.channelSamples[0][0].sampleIndex)
            + ":"
            + str(data.channelSamples[0][0].data)
            + ":"
            + str(data.channelSamples[0][0].impedance)
        )

        # for sample in data.channelSamples[0]:
        #     print(sample.data)
    pass


def onPowerChanged(sensor: SensorProfile, power: int):
    print("connected sensor: " + sensor.BLEDevice.Name + " power: " + str(power))
    # if not sensor.isDataTransfering:
    #     print("do disconnect")
    #     sensor.disconnect()


def onStateChanged(sensor: SensorProfile, newstate: DeviceStateEx):
    print("device: " + sensor.BLEDevice.Name + str(newstate))


def onErrorCallback(sensor: SensorProfile, reason: str):
    print("device: " + sensor.BLEDevice.Name + reason)
    pass


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
