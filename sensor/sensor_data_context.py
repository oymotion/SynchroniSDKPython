import asyncio
from collections import deque
from queue import Queue
from typing import Deque, List

from sensor.gforce import DataSubscription, GForce
from sensor.sensor_data import DataType, Sample, SensorData

from enum import Enum, IntEnum

from sensor.sensor_device import DeviceInfo
from sensor.utils import timer


class SensorDataType(IntEnum):
    DATA_TYPE_EEG = 0
    DATA_TYPE_ECG = 1
    DATA_TYPE_ACC = 2
    DATA_TYPE_GYRO = 3
    DATA_TYPE_BRTH = 4
    DATA_TYPE_COUNT = 5


# 枚举 FeatureMaps 的 Python 实现
class FeatureMaps(Enum):
    GFD_FEAT_EMG = 0x000002000
    GFD_FEAT_EEG = 0x000400000
    GFD_FEAT_ECG = 0x000800000
    GFD_FEAT_IMPEDANCE = 0x001000000
    GFD_FEAT_IMU = 0x002000000
    GFD_FEAT_ADS = 0x004000000
    GFD_FEAT_BRTH = 0x008000000
    GFD_FEAT_CONCAT_BLE = 0x80000000


class SensorProfileDataCtx:
    def __init__(self, gForce: GForce, deviceMac: str, buf: Queue[bytes]):
        self.featureMap = 0
        self.notifyDataFlag: DataSubscription = 0

        self.gForce = gForce
        self.deviceMac = deviceMac
        self._device_info: DeviceInfo = None

        self._is_running = True
        self._is_data_transfering = False
        self.isUniversalStream: bool = gForce._is_universal_stream
        self._rawDataBuffer: Queue[bytes] = buf
        self._concatDataBuffer = bytearray()

        self.sensorDatas: List[SensorData] = list()
        for idx in range(0, SensorDataType.DATA_TYPE_COUNT):
            self.sensorDatas.append(SensorData())
        self.impedanceData: List[float] = list()
        self.saturationData: List[float] = list()

    def close(self):
        self._is_running = False

    def clear(self):
        for sensorData in self.sensorDatas:
            sensorData.clear()
        self.impedanceData.clear()
        self.saturationData.clear()
        self._concatDataBuffer.clear()
        self._rawDataBuffer.queue.clear()

    def reset(self):
        self.notifyDataFlag = 0
        self.clear()

    @property
    def isDataTransfering(self) -> bool:
        """
        检查传感器是否正在进行数据传输。

        :return:            bool: 如果传感器正在进行数据传输，返回 True；否则返回 False。
        """
        return self._is_data_transfering

    def hasInit(self):
        return self.featureMap != 0 and self.notifyDataFlag != 0

    def hasEMG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_EMG.value) != 0

    def hasEEG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_EEG.value) != 0

    def hasECG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_ECG.value) != 0

    def hasImpedance(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_IMPEDANCE.value) != 0

    def hasIMU(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_IMU.value) != 0

    def hasBrth(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_BRTH.value) != 0

    def hasConcatBLE(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_CONCAT_BLE.value) != 0

    async def initEEG(self, packageCount: int) -> int:
        config = await self.gForce.get_eeg_raw_data_config()
        cap = await self.gForce.get_eeg_raw_data_cap()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_EEG
        data.sampleRate = config.fs
        data.resolutionBits = config.resolution
        data.channelCount = cap.channel_count
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.K
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_EEG] = data
        self.notifyDataFlag |= DataSubscription.DNF_EEG
        return data.channelCount

    async def initECG(self, packageCount: int) -> int:
        config = await self.gForce.get_ecg_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ECG
        data.sampleRate = config.fs
        data.resolutionBits = config.resolution
        data.channelCount = 1
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.K
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ECG] = data
        self.notifyDataFlag |= DataSubscription.DNF_ECG
        return data.channelCount

    async def initIMU(self, packageCount: int) -> int:
        config = await self.gForce.get_imu_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ACC
        data.sampleRate = config.fs
        data.resolutionBits = 16
        data.channelCount = config.channel_count
        data.channelMask = 255
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.accK
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ACC] = data

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_GYRO
        data.sampleRate = config.fs
        data.resolutionBits = 16
        data.channelCount = config.channel_count
        data.channelMask = 255
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.gyroK
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GYRO] = data

        self.notifyDataFlag |= DataSubscription.DNF_IMU

        return data.channelCount

    async def initBrth(self, packageCount: int) -> int:
        config = await self.gForce.get_brth_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_BRTH
        data.sampleRate = config.fs
        data.resolutionBits = config.resolution
        data.channelCount = 1
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.K
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_BRTH] = data
        self.notifyDataFlag |= DataSubscription.DNF_ECG
        return data.channelCount

    async def initDataTransfer(self, isGetFeature: bool) -> int:
        if isGetFeature:
            self.featureMap = await self.gForce.get_feature_map()
            if self.hasImpedance():
                self.notifyDataFlag |= DataSubscription.DNF_IMPEDANCE
            return self.featureMap
        else:
            await self.gForce.set_subscription(self.notifyDataFlag)
            return self.notifyDataFlag

    async def fetchDeviceInfo(self) -> DeviceInfo:
        info = DeviceInfo()
        info.MTUSize = self.gForce.client.mtu_size
        info.DeviceName = await self.gForce.get_device_name()
        info.ModelName = await self.gForce.get_model_number()
        info.HardwareVersion = await self.gForce.get_hardware_revision()
        info.FirmwareVersion = await self.gForce.get_firmware_revision()
        return info

    async def init(self, packageCount: int) -> bool:
        try:
            info = await self.fetchDeviceInfo()

            await self.initDataTransfer(True)

            if self.hasImpedance():
                self.notifyDataFlag |= DataSubscription.DNF_IMPEDANCE

            if self.hasEEG():
                info.EegChannelCount = await self.initEEG(packageCount)

            if self.hasECG():
                info.EcgChannelCount = await self.initECG(packageCount)

            if self.hasBrth():
                info.BrthChannelCount = await self.initBrth(packageCount)

            if self.hasIMU():
                imuChannelCount = await self.initIMU(packageCount)
                info.AccChannelCount = imuChannelCount
                info.GyroChannelCount = imuChannelCount

            self._device_info = info

            if not self.isUniversalStream:
                await self.initDataTransfer(False)

            return True
        except Exception as e:
            print(e)
            return False

    async def start_streaming(self) -> bool:
        if self._is_data_transfering:
            return True
        self._is_data_transfering = True
        self._rawDataBuffer.queue.clear()
        if not self.isUniversalStream:
            await self.gForce.start_streaming(self._rawDataBuffer)
            return True
        else:
            await self.gForce.set_subscription(self.notifyDataFlag)
            return True

    async def stop_streaming(self) -> bool:
        if not self._is_data_transfering:
            return True

        self._is_data_transfering = False

        if not self.isUniversalStream:
            await self.gForce.stop_streaming()
            return True
        else:
            await self.gForce.set_subscription(0)
            return True

    def process_data(self, buf: Queue[SensorData]):
        try:
            data: bytes = self._rawDataBuffer.get_nowait()
        except Exception as e:
            return

        self._processDataPackage(data, buf)
        self._rawDataBuffer.task_done()

    def _processDataPackage(self, data: bytes, buf: Queue[SensorData]):
        v = data[0]
        if v == DataType.NTF_IMPEDANCE:
            offset = 1
            # packageIndex = ((data[offset + 1] & 0xff) << 8) | (data[offset] & 0xff)
            offset += 2

            impedanceData = []
            saturationData = []

            dataCount = (len(data) - 3) // 4 // 2
            for index in range(dataCount):
                impedance_bytes = data[offset : offset + 4]
                impedance = int.from_bytes(impedance_bytes, byteorder="little")
                offset += 4
                impedanceData.append(impedance)

            for index in range(dataCount):
                saturation_bytes = data[offset : offset + 4]
                saturation = int.from_bytes(saturation_bytes, byteorder="little")
                offset += 4
                saturationData.append(saturation / 10)  # firmware value range 0 - 1000

            self.impedanceData = impedanceData
            self.saturationData = saturationData

        elif v == DataType.NTF_EEG:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_EEG]
            if self.checkReadSamples(data, sensor_data, 3, 0):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_ECG:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_ECG]
            if self.checkReadSamples(data, sensor_data, 3, 0):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_BRTH:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_BRTH]
            if self.checkReadSamples(data, sensor_data, 3, 0):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_IMU:
            sensor_data_acc = self.sensorDatas[SensorDataType.DATA_TYPE_ACC]
            if self.checkReadSamples(data, sensor_data_acc, 3, 6):
                self.sendSensorData(sensor_data_acc, buf)

            sensor_data_gyro = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO]
            if self.checkReadSamples(data, sensor_data_gyro, 9, 6):
                self.sendSensorData(sensor_data_gyro, buf)

    def checkReadSamples(
        self, data: bytes, sensorData: SensorData, dataOffset: int, dataGap: int
    ):
        offset = 1
        v = data[0]
        if not self._is_data_transfering:
            return False
        try:

            packageIndex = ((data[offset + 1] & 0xFF) << 8) | (data[offset] & 0xFF)
            offset += 2
            newPackageIndex = packageIndex
            lastPackageIndex = sensorData.lastPackageIndex

            if packageIndex < lastPackageIndex:
                packageIndex += 65536  # 包索引是 U16 类型
            elif packageIndex == lastPackageIndex:
                return False

            deltaPackageIndex = packageIndex - lastPackageIndex
            if deltaPackageIndex > 1:
                lostSampleCount = sensorData.packageSampleCount * (
                    deltaPackageIndex - 1
                )
                print(
                    f"lost dataType {sensorData.dataType} -> data {sensorData.deviceMac} {lostSampleCount}"
                )
                self.readSamples(data, sensorData, 0, dataGap, lostSampleCount)
                if newPackageIndex == 0:
                    sensorData.lastPackageIndex = 65535
                else:
                    sensorData.lastPackageIndex = newPackageIndex - 1
                sensorData.lastPackageCounter += deltaPackageIndex - 1

            self.readSamples(data, sensorData, dataOffset, dataGap, 0)
            sensorData.lastPackageIndex = newPackageIndex
            sensorData.lastPackageCounter += 1
        except Exception:
            return False
        return True

    def readSamples(
        self,
        data: bytes,
        sensorData: SensorData,
        offset: int,
        dataGap: int,
        lostSampleCount: int,
    ):
        sampleCount = sensorData.packageSampleCount
        sampleInterval = 1000 // sensorData.sampleRate
        if lostSampleCount > 0:
            sampleCount = lostSampleCount

        K = sensorData.K
        lastSampleIndex = sensorData.lastPackageCounter * sensorData.packageSampleCount

        _impedanceData = self.impedanceData.copy()
        _saturationData = self.saturationData.copy()

        channelSamples = sensorData.channelSamples
        if not channelSamples:
            for channelIndex in range(sensorData.channelCount):
                channelSamples.append([])

        for sampleIndex in range(sampleCount):
            for channelIndex, impedanceChannelIndex in enumerate(
                range(sensorData.channelCount)
            ):
                if (sensorData.channelMask & (1 << channelIndex)) != 0:
                    samples = channelSamples[channelIndex]
                    impedance = 0.0
                    saturation = 0.0
                    if sensorData.dataType == DataType.NTF_ECG:
                        impedanceChannelIndex = self.sensorDatas[
                            SensorDataType.DATA_TYPE_EEG
                        ].channelCount
                    impedance = _impedanceData[impedanceChannelIndex]
                    saturation = _saturationData[impedanceChannelIndex]
                    impedanceChannelIndex += 1

                    dataItem = Sample()
                    dataItem.channelIndex = channelIndex
                    dataItem.sampleIndex = lastSampleIndex
                    dataItem.timeStampInMs = lastSampleIndex * sampleInterval
                    if lostSampleCount > 0:
                        dataItem.rawData = 0
                        dataItem.data = 0.0
                        dataItem.impedance = impedance
                        dataItem.saturation = saturation
                        dataItem.isLost = True
                    else:
                        rawData = 0
                        if sensorData.resolutionBits == 8:
                            rawData = data[offset]
                            rawData -= 128
                            offset += 1
                        elif sensorData.resolutionBits == 16:
                            rawData = int.from_bytes(
                                data[offset : offset + 2],
                                byteorder="little",
                                signed=True,
                            )
                            offset += 2
                        elif sensorData.resolutionBits == 24:
                            rawData = (
                                (data[offset] << 16)
                                | (data[offset + 1] << 8)
                                | data[offset + 2]
                            )
                            rawData -= 8388608
                            offset += 3

                        converted = rawData * K
                        dataItem.rawData = rawData
                        dataItem.data = converted
                        dataItem.impedance = impedance
                        dataItem.saturation = saturation
                        dataItem.isLost = False

                    samples.append(dataItem)

            lastSampleIndex += 1
            offset += dataGap

    def sendSensorData(self, sensorData: SensorData, buf: Queue[SensorData]):
        oldChannelSamples = sensorData.channelSamples

        if not self.isDataTransfering or len(oldChannelSamples) == 0:
            return

        realSampleCount = 0
        if len(oldChannelSamples) > 0:
            realSampleCount = len(oldChannelSamples[0])

        if realSampleCount < sensorData.minPackageSampleCount:
            return

        sensorData.channelSamples = []
        batchCount = realSampleCount // sensorData.minPackageSampleCount
        # leftSampleSize = realSampleCount - sensorData.minPackageSampleCount * batchCount

        sensorDataList = []
        startIndex = 0
        for batchIndex in range(batchCount):
            resultChannelSamples = []
            for channelIndex in range(sensorData.channelCount):
                oldSamples = oldChannelSamples[channelIndex]
                newSamples = []
                for sampleIndex in range(sensorData.minPackageSampleCount):
                    newSamples.append(oldSamples[startIndex + sampleIndex])
                resultChannelSamples.append(newSamples)

            sensorDataResult = SensorData()
            sensorDataResult.channelSamples = resultChannelSamples
            sensorDataResult.dataType = sensorData.dataType
            sensorDataResult.deviceMac = sensorData.deviceMac
            sensorDataResult.sampleRate = sensorData.sampleRate
            sensorDataResult.channelCount = sensorData.channelCount
            sensorDataResult.minPackageSampleCount = sensorData.minPackageSampleCount
            sensorDataList.append(sensorDataResult)

            startIndex += sensorData.minPackageSampleCount

        leftChannelSamples = []
        for channelIndex in range(sensorData.channelCount):
            oldSamples = oldChannelSamples[channelIndex]
            newSamples = []
            for sampleIndex in range(startIndex, len(oldSamples)):
                newSamples.append(oldSamples[sampleIndex])

            leftChannelSamples.append(newSamples)

        sensorData.channelSamples = leftChannelSamples

        for sensorDataResult in sensorDataList:
            buf.put(sensorDataResult)

    def calc_crc8(self, data):
        crc8Table = [
            0x00,
            0x07,
            0x0E,
            0x09,
            0x1C,
            0x1B,
            0x12,
            0x15,
            0x38,
            0x3F,
            0x36,
            0x31,
            0x24,
            0x23,
            0x2A,
            0x2D,
            0x70,
            0x77,
            0x7E,
            0x79,
            0x6C,
            0x6B,
            0x62,
            0x65,
            0x48,
            0x4F,
            0x46,
            0x41,
            0x54,
            0x53,
            0x5A,
            0x5D,
            0xE0,
            0xE7,
            0xEE,
            0xE9,
            0xFC,
            0xFB,
            0xF2,
            0xF5,
            0xD8,
            0xDF,
            0xD6,
            0xD1,
            0xC4,
            0xC3,
            0xCA,
            0xCD,
            0x90,
            0x97,
            0x9E,
            0x99,
            0x8C,
            0x8B,
            0x82,
            0x85,
            0xA8,
            0xAF,
            0xA6,
            0xA1,
            0xB4,
            0xB3,
            0xBA,
            0xBD,
            0xC7,
            0xC0,
            0xC9,
            0xCE,
            0xDB,
            0xDC,
            0xD5,
            0xD2,
            0xFF,
            0xF8,
            0xF1,
            0xF6,
            0xE3,
            0xE4,
            0xED,
            0xEA,
            0xB7,
            0xB0,
            0xB9,
            0xBE,
            0xAB,
            0xAC,
            0xA5,
            0xA2,
            0x8F,
            0x88,
            0x81,
            0x86,
            0x93,
            0x94,
            0x9D,
            0x9A,
            0x27,
            0x20,
            0x29,
            0x2E,
            0x3B,
            0x3C,
            0x35,
            0x32,
            0x1F,
            0x18,
            0x11,
            0x16,
            0x03,
            0x04,
            0x0D,
            0x0A,
            0x57,
            0x50,
            0x59,
            0x5E,
            0x4B,
            0x4C,
            0x45,
            0x42,
            0x6F,
            0x68,
            0x61,
            0x66,
            0x73,
            0x74,
            0x7D,
            0x7A,
            0x89,
            0x8E,
            0x87,
            0x80,
            0x95,
            0x92,
            0x9B,
            0x9C,
            0xB1,
            0xB6,
            0xBF,
            0xB8,
            0xAD,
            0xAA,
            0xA3,
            0xA4,
            0xF9,
            0xFE,
            0xF7,
            0xF0,
            0xE5,
            0xE2,
            0xEB,
            0xEC,
            0xC1,
            0xC6,
            0xCF,
            0xC8,
            0xDD,
            0xDA,
            0xD3,
            0xD4,
            0x69,
            0x6E,
            0x67,
            0x60,
            0x75,
            0x72,
            0x7B,
            0x7C,
            0x51,
            0x56,
            0x5F,
            0x58,
            0x4D,
            0x4A,
            0x43,
            0x44,
            0x19,
            0x1E,
            0x17,
            0x10,
            0x05,
            0x02,
            0x0B,
            0x0C,
            0x21,
            0x26,
            0x2F,
            0x28,
            0x3D,
            0x3A,
            0x33,
            0x34,
            0x4E,
            0x49,
            0x40,
            0x47,
            0x52,
            0x55,
            0x5C,
            0x5B,
            0x76,
            0x71,
            0x78,
            0x7F,
            0x6A,
            0x6D,
            0x64,
            0x63,
            0x3E,
            0x39,
            0x30,
            0x37,
            0x22,
            0x25,
            0x2C,
            0x2B,
            0x06,
            0x01,
            0x08,
            0x0F,
            0x1A,
            0x1D,
            0x14,
            0x13,
            0xAE,
            0xA9,
            0xA0,
            0xA7,
            0xB2,
            0xB5,
            0xBC,
            0xBB,
            0x96,
            0x91,
            0x98,
            0x9F,
            0x8A,
            0x8D,
            0x84,
            0x83,
            0xDE,
            0xD9,
            0xD0,
            0xD7,
            0xC2,
            0xC5,
            0xCC,
            0xCB,
            0xE6,
            0xE1,
            0xE8,
            0xEF,
            0xFA,
            0xFD,
            0xF4,
            0xF3,
        ]
        crc8 = 0
        len_data = len(data)

        for i in range(len_data):
            crc8 ^= data[i]
            crc8 = crc8Table[crc8]

        return crc8

    def processUniversalData(
        self, buf: Queue[SensorData], loop: asyncio.AbstractEventLoop, sensor, callback
    ):

        while self._is_running:
            try:
                while self._is_running and not self._rawDataBuffer.empty():
                    data = self._rawDataBuffer.get_nowait()
                    self._concatDataBuffer.extend(data)
                    self._rawDataBuffer.task_done()
            except Exception as e:
                pass

            index = 0
            last_cut = -1

            while self._is_running:
                data_size = len(self._concatDataBuffer)
                if index >= data_size:
                    break

                if self._concatDataBuffer[index] == 0x55:
                    if (index + 1) >= data_size:
                        index += 1
                        continue
                    n = self._concatDataBuffer[index + 1]
                    if (index + 1 + n + 1) >= data_size:
                        index += 1
                        continue
                    crc = self._concatDataBuffer[index + 1 + n + 1]
                    calc_crc = self.calc_crc8(
                        self._concatDataBuffer[index + 2 : index + 2 + n]
                    )
                    if crc != calc_crc:
                        index += 1
                        continue
                    if self._is_data_transfering:
                        data_package = bytes(
                            self._concatDataBuffer[index + 2 : index + 2 + n]
                        )
                        self._processDataPackage(data_package, buf)
                        while self._is_running and self.isDataTransfering:
                            sensorData: SensorData = None
                            try:
                                sensorData = buf.get_nowait()
                            except Exception as e:
                                break
                            if sensorData != None and callback != None:
                                try:
                                    loop.call_soon_threadsafe(
                                        callback, sensor, sensorData
                                    )
                                except Exception as e:
                                    print(e)

                            buf.task_done()

                    last_cut = index = index + 2 + n
                elif self._concatDataBuffer[index] == 0xAA:
                    if (index + 1) >= data_size:
                        index += 1
                        continue
                    n = self._concatDataBuffer[index + 1]
                    if (index + 1 + n + 1) >= data_size:
                        index += 1
                        continue
                    crc = self._concatDataBuffer[index + 1 + n + 1]
                    calc_crc = self.calc_crc8(
                        self._concatDataBuffer[index + 2 : index + 2 + n]
                    )
                    if crc != calc_crc:
                        index += 1
                        continue
                    data_package = bytes(
                        self._concatDataBuffer[index + 2 : index + 2 + n]
                    )
                    loop.call_soon_threadsafe(
                        self.gForce._on_cmd_response, None, data_package
                    )
                    last_cut = index = index + 2 + n
                else:
                    index += 1

            if last_cut > 0:
                self._concatDataBuffer = self._concatDataBuffer[last_cut + 1 :]
