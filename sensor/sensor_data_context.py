import asyncio
from collections import deque
import os
import platform
from queue import Queue
import struct
import math
from typing import Deque, List
from concurrent.futures import ThreadPoolExecutor
import csv
from sensor import sensor_utils
from sensor.gforce import DataSubscription, GForce, SamplingRate
from sensor.sensor_data import DataType, Sample, SensorData

from enum import Enum, IntEnum

from sensor.sensor_device import DeviceInfo


class SensorDataType(IntEnum):
    DATA_TYPE_EEG = 0
    DATA_TYPE_ECG = 1
    DATA_TYPE_ACC = 2
    DATA_TYPE_GYRO = 3
    DATA_TYPE_BRTH = 4
    DATA_TYPE_EMG = 5
    DATA_TYPE_MAG_ANGLE = 6
    DATA_TYPE_QUATERNION = 7
    DATA_TYPE_PPG = 8
    DATA_TYPE_SPO2 = 10
    DATA_TYPE_EULER = 9
    DATA_TYPE_GFORCE_QUAT = 11
    DATA_TYPE_COUNT = 12



class FeatureMaps(Enum):
    GFD_FEAT_EMG = 0x000002000
    GFD_FEAT_MAGANG = 0x00080000
    GFD_FEAT_EEG = 0x000400000
    GFD_FEAT_ECG = 0x000800000
    GFD_FEAT_IMPEDANCE = 0x001000000
    GFD_FEAT_IMU = 0x002000000
    GFD_FEAT_ADS = 0x004000000
    GFD_FEAT_BRTH = 0x008000000
    GFD_FEAT_CONCAT_BLE = 0x80000000
    GFD_FEAT_PPG = 0x10000000
    GFD_FEAT_EULER = 0x000000200
    GFD_FEAT_QUAT = 0x000000400
    GFD_FEAT_ACC = 0x000000040
    GFD_FEAT_GYRO = 0x000000080


class PPGDataMode(IntEnum):

    SPO2_AND_HR = 0
    PPG_RAW = 1
    PPG_AND_SPO2 = 2


class SensorProfileDataCtx:
    def __init__(self, gForce: GForce, deviceMac: str, buf: Queue[bytes]):
        self.featureMap = 0
        self.notifyDataFlag: DataSubscription = 0

        self.gForce = gForce
        self.deviceMac = deviceMac
        self._device_info: DeviceInfo = None

        self._is_initing = False
        self._is_running = True
        self._is_data_transfering = False
        self.isUniversalStream: bool = gForce._is_universal_stream
        self._rawDataBuffer: Queue[bytes] = buf
        self._concatDataBuffer = bytearray()

        # Quaternion support
        self.isContainQAT6 = False
        self.lastQuatData: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.lastEulerData: List[float] = [0.0, 0.0, 0.0]
        self.lastAccData: List[float] = [0.0, 0.0, 0.0]
        self.lastGyroData: List[float] = [0.0, 0.0, 0.0]
        # PPG configuration
        self.model = PPGDataMode.PPG_RAW

        self.sensorDatas: List[SensorData] = list()
        for idx in range(0, SensorDataType.DATA_TYPE_COUNT):
            self.sensorDatas.append(SensorData())
        self.impedanceData: List[float] = list()
        self.saturationData: List[float] = list()
        self.dataPool = ThreadPoolExecutor(1, "data")
        self.init_map = {"NTF_EMG": "ON", "NTF_EEG": "ON", "NTF_ECG": "ON", "NTF_IMU": "ON", "NTF_BRTH": "ON",
                         "NTF_MAG_ANGLE": "ON", "NTF_IMPEDANCE": "ON", "NTF_PPG_RAW": "ON", "NTF_SPO2": "ON",
                         "NTF_GFORCE_EULER": "ON",
                         "NTF_GFORCE_QUAT": "ON",
                         "NTF_GFORCE_ACC": "ON",
                         "NTF_GFORCE_GYRO": "ON",
                         }
        self.filter_map = {"FILTER_50HZ": "ON", "FILTER_60HZ": "ON", "FILTER_HPF": "ON", "FILTER_LPF": "ON"}
        self.debugCSVWriter = None
        self.debugCSVPath = None

    def close(self):  
        self._is_running = False
        if self.debugCSVWriter != None:
            self.debugCSVWriter = None

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

        return self._is_data_transfering

    def hasInit(self):
        return not self._is_initing and self.featureMap != 0 and self.notifyDataFlag != 0

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

    def hasMagAngle(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_MAGANG.value) != 0

    def hasConcatBLE(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_CONCAT_BLE.value) != 0

    def hasPPG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_PPG.value) != 0

    def hasEuler(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_EULER.value) != 0

    def hasQuat(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_QUAT.value) != 0

    def hasAcc(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_ACC.value) != 0

    def hasGyro(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_GYRO.value) != 0
    


    async def initEMG(self, packageCount: int) -> int:
        config = await self.gForce.get_emg_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_EMG
        data.sampleRate = 500
        data.resolutionBits = 0
        data.channelCount = 8
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = 8

        data.clear()
        isNewEMG = True
        try:
            if self._device_info.DeviceName.startswith("gForce") or self._device_info.DeviceName.startswith(
                    "OHand") or self._device_info.DeviceName.startswith(
                    "ORE-") or self._device_info.DeviceName.startswith(
                    "OYEM-") or self._device_info.DeviceName.startswith("ORehab"):
                isNewEMG = False
        except Exception as e:
            pass

        if (isNewEMG):
            # new emg
            data.packageIndexLength = 2
            data.resolutionBits = 0
            gain = 6
            data.K = 4000000.0 / 8388607.0 / gain
            config.resolution = 8
        else:
            # old emg
            data.packageIndexLength = 1
            data.resolutionBits = 7
            gain = 1200
            min_voltage = -1.25 * 1000000
            max_voltage = 1.25 * 100000
            div = 127.0
            conversion_factor = (max_voltage - min_voltage) / gain / div
            data.K = conversion_factor
            config.resolution = 8

        config.fs = SamplingRate.HZ_500
        config.channel_mask = 255
        config.batch_len = 128

        if isNewEMG:
            await self.gForce.set_function_switch(2)
    
        await self.gForce.set_emg_raw_data_config(config)
        await self.gForce.set_package_id(True)

        if isNewEMG:
            if self.notifyDataFlag & DataSubscription.DNF_CONCAT_BLE != 0:
                data.packageSampleCount = 15
            else:
                data.packageSampleCount = 8

        self.sensorDatas[SensorDataType.DATA_TYPE_EMG] = data
        self.notifyDataFlag |= DataSubscription.EMG_RAW

        return data.channelCount

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
    async def initPPG(self, packageCount: int) -> int:


        config = await self.gForce.get_ppg_raw_data_config()
        

        config.fs = self.sampleRate
        config.mode = self.model
        

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.sampleRate = config.fs
        data.channelMask = 255
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = 1.0
        if self.model == PPGDataMode.PPG_RAW:

            data.dataType = DataType.NTF_PPG
            data.resolutionBits = 24
            data.channelCount = 2
            self.sensorDatas[SensorDataType.DATA_TYPE_PPG] = data
        elif self.model == PPGDataMode.SPO2_AND_HR:
            data.dataType = DataType.NTF_SPO2
            config.period = self.period
            config.fs = SamplingRate.HZ_100
            data.resolutionBits = 16
            data.channelCount = 2
            self.sensorDatas[SensorDataType.DATA_TYPE_SPO2] = data
        else:

            data.dataType = DataType.NTF_PPG
            data.resolutionBits = 24
            data.channelCount = 2
            self.sensorDatas[SensorDataType.DATA_TYPE_PPG] = data
            data.dataType = DataType.NTF_SPO2
            config.period = self.period
            config.fs = self.sampleRate
            data.resolutionBits = 16
            data.channelCount = 1
            self.sensorDatas[SensorDataType.DATA_TYPE_SPO2] = data

        await self.gForce.set_ppg_raw_data_config(config)
        self.notifyDataFlag=DataSubscription.DNF_PPG;
        data.clear()
        
        return data.channelCount

    def setPPGMode(self, mode: int, sampleRate: int = None, period: int = None):

        if mode not in [PPGDataMode.SPO2_AND_HR, PPGDataMode.PPG_RAW]:
            raise ValueError(f"Invalid PPG mode: {mode}. Must be 0 (SPO2AndHR) or 1 (PPGRaw)")
        
        self.model = mode
        self.period = period
        self.sampleRate = sampleRate
        
        print(f"PPG mode set to: {'SPO2AndHR' if mode == 0 else 'PPGRaw'}, Sample rate: {self.sampleRate}Hz")


    async def initIMU(self, packageCount: int) -> int:
        IMU_TYPE_QAT6 = 0x0004
        min_package_sample_count = 2
        self.isContainQAT6 = False
        config = await self.gForce.get_imu_raw_data_config()
        imu_cap = await self.gForce.get_imu_cap_data_config()
        if imu_cap is not None:
            channel_count, samp_rate, sample_count = imu_cap
            if (channel_count & IMU_TYPE_QAT6) == IMU_TYPE_QAT6:
                self.isContainQAT6 = True
                config.channel_count = channel_count
                await self.gForce.set_imu_raw_data_config(config)
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ACC
        data.sampleRate = config.fs
        data.resolutionBits = 16
        data.channelCount = 3
        data.channelMask = 255
        data.minPackageSampleCount = min_package_sample_count
        data.packageSampleCount = config.batch_len
        data.K = config.accK
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ACC] = data

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_GYRO
        data.sampleRate = config.fs
        data.resolutionBits = 16
        data.channelCount = 3
        data.channelMask = 255
        data.minPackageSampleCount = min_package_sample_count
        data.packageSampleCount = config.batch_len
        data.K = config.gyroK
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GYRO] = data

        # Initialize quaternion data if supported
        if self.isContainQAT6:
            data = SensorData()
            data.deviceMac = self.deviceMac
            data.dataType = DataType.NTF_QUATERNION
            data.sampleRate = config.fs
            data.resolutionBits = 32
            data.channelCount = 4  # w, x, y, z
            data.channelMask = 15  # 0b1111
            data.minPackageSampleCount = min_package_sample_count
            data.packageSampleCount = config.batch_len
            data.K = 1.0 / 1073741824.0  # 1 / 2^30
            data.clear()
            self.sensorDatas[SensorDataType.DATA_TYPE_QUATERNION] = data

            data = SensorData()
            data.deviceMac = self.deviceMac
            data.dataType = DataType.NTF_EULER_DATA
            data.sampleRate = config.fs
            data.resolutionBits = 32        # float32
            data.channelCount = 3           # pitch, roll, yaw
            data.channelMask = 0b0111
            data.packageIndexLength = 0
            data.minPackageSampleCount = min_package_sample_count
            data.packageSampleCount = config.batch_len
            data.K = 1.0
            data.clear()
            self.sensorDatas[SensorDataType.DATA_TYPE_EULER] = data

        self.notifyDataFlag |= DataSubscription.DNF_IMU

        return config.channel_count

    async def initEuler(self, packageCount: int) -> int:

        # config = await self.gForce.get_imu_raw_data_config()

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_EULER_DATA
        data.sampleRate = 40
        data.resolutionBits = 32        # float32
        data.channelCount = 3           # roll, pitch, yaw
        data.channelMask = 0b0111
        data.packageIndexLength = 0
        data.minPackageSampleCount = 2
        data.packageSampleCount = 1
        data.K = 1.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_EULER] = data
        self.notifyDataFlag |= DataSubscription.EULERANGLE
        return data.channelCount

    async def initGForceQuat(self, packageCount: int) -> int:

        # config = await self.gForce.get_imu_raw_data_config()

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_QUATERNION
        data.sampleRate = 40
        data.resolutionBits = 32        # float32
        data.channelCount = 4           # w, x, y, z
        data.channelMask = 0b1111
        data.packageIndexLength = 0
        data.minPackageSampleCount = 2
        data.packageSampleCount = 1
        data.K = 1.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GFORCE_QUAT] = data
        self.notifyDataFlag |= DataSubscription.QUATERNION
        return data.channelCount

    async def initGForceAcc(self, packageCount: int) -> int:

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ACC
        data.sampleRate = 40
        data.resolutionBits = 32        # int32
        data.channelCount = 3           # x, y, z
        data.channelMask = 0b0111
        data.packageIndexLength = 1
        data.minPackageSampleCount = 2
        data.packageSampleCount = 1
        data.K = 1.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ACC] = data
        self.notifyDataFlag |= DataSubscription.ACCELERATE
        return data.channelCount

    async def initGForceGyro(self, packageCount: int) -> int:

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_GYRO
        data.sampleRate = 40
        data.resolutionBits = 32        # int32
        data.channelCount = 3           # x, y, z
        data.channelMask = 0b0111
        data.packageIndexLength = 1
        data.minPackageSampleCount = 2
        data.packageSampleCount = 1
        data.K = 1.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GYRO] = data
        self.notifyDataFlag |= DataSubscription.GYROSCOPE
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

    async def initMagAngle(self, packageCount: int) -> int:
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_MAG_ANGLE_DATA
        data.sampleRate = 40
        data.resolutionBits = 8
        data.channelCount = 1
        data.channelMask = 1
        data.minPackageSampleCount = 2
        data.packageSampleCount = 1
        data.K = 1
        data.packageIndexLength = 2
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_MAG_ANGLE] = data
        self.notifyDataFlag |= DataSubscription.DNF_MAG_ANGLE_EXT
        return data.channelCount

    async def initDataTransfer(self, isGetFeature: bool) -> int:
        if isGetFeature:
            self.featureMap = await self.gForce.get_feature_map()
            return self.featureMap
        else:
            await self.gForce.set_subscription(self.notifyDataFlag)
            return self.notifyDataFlag

    async def fetchDeviceInfo(self) -> DeviceInfo:
        info = DeviceInfo()
        if platform.system() != "Linux":
            info.MTUSize = self.gForce.client.mtu_size
        else:
            info.MTUSize = 0
        # print("get_device_name")
        info.DeviceName = await self.gForce.get_device_name()
        # print("get_model_number")
        info.ModelName = await self.gForce.get_model_number()
        # print("get_hardware_revision")
        info.HardwareVersion = await self.gForce.get_hardware_revision()
        # print("get_firmware_revision")
        info.FirmwareVersion = await self.gForce.get_firmware_revision()
        return info

    async def init(self, packageCount: int) -> bool:
        if self._is_initing:
            return False
        try:
            self._is_initing = True
            info = await self.fetchDeviceInfo()
            self._device_info = info
            await self.initDataTransfer(True)

            if self.hasConcatBLE():
                self.notifyDataFlag |= DataSubscription.DNF_CONCAT_BLE

            if self.hasImpedance() and (self.init_map["NTF_IMPEDANCE"] == "ON"):
                self.notifyDataFlag |= DataSubscription.DNF_IMPEDANCE

            if self.hasEMG() and (self.init_map["NTF_EMG"] == "ON"):
                info.EmgChannelCount = await self.initEMG(packageCount)
                info.EmgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EMG].sampleRate

            if self.hasEEG() and (self.init_map["NTF_EEG"] == "ON"):
                info.EegChannelCount = await self.initEEG(packageCount)
                info.EegSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EEG].sampleRate

            if self.hasECG() and (self.init_map["NTF_ECG"] == "ON"):
                info.EcgChannelCount = await self.initECG(packageCount)
                info.EcgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ECG].sampleRate

            if self.hasBrth() and (self.init_map["NTF_BRTH"] == "ON"):
                info.BrthChannelCount = await self.initBrth(packageCount)
                info.BrthSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_BRTH].sampleRate

            if self.hasIMU() and (self.init_map["NTF_IMU"] == "ON"):
                await self.initIMU(packageCount)
                info.AccChannelCount = 3
                info.GyroChannelCount = 3
                info.AccSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ACC].sampleRate
                info.GyroSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO].sampleRate
                if self.isContainQAT6:
                    info.QuatChannelCount = 4
                    info.EulerChannelCount = 3
                    info.QuatSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_QUATERNION].sampleRate
                    info.EulerSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EULER].sampleRate

            if self.hasEuler() and (self.init_map["NTF_GFORCE_EULER"] == "ON"):
                info.EulerChannelCount = await self.initEuler(packageCount)
                info.EulerSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EULER].sampleRate


            if self.hasQuat() and (self.init_map["NTF_GFORCE_QUAT"] == "ON"):
                info.QuatChannelCount = await self.initGForceQuat(packageCount)
                info.QuatSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_GFORCE_QUAT].sampleRate


            if self.hasAcc() and (self.init_map["NTF_GFORCE_ACC"] == "ON"):
                info.AccChannelCount = await self.initGForceAcc(packageCount)
                info.AccSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ACC].sampleRate


            if self.hasGyro() and (self.init_map["NTF_GFORCE_GYRO"] == "ON"):
                info.GyroChannelCount = await self.initGForceGyro(packageCount)
                info.GyroSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO].sampleRate
            if self.hasPPG() and (self.init_map["NTF_PPG_RAW"] == "ON"):
                info.PpgChannelCount = await self.initPPG(packageCount)

                if self.model == PPGDataMode.PPG_RAW:

                    info.Spo2SampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2].sampleRate
                if self.model == PPGDataMode.SPO2_AND_HR:

                    info.PpgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2].sampleRate
                else:
                    info.PpgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_PPG].sampleRate
                    info.Spo2SampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2].sampleRate
            if self.hasMagAngle() and (self.init_map["NTF_MAG_ANGLE"] == "ON"):
                magAngleChannelCount = await self.initMagAngle(packageCount)
                info.MagAngleChannelCount = magAngleChannelCount
                info.MagAngleSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_MAG_ANGLE].sampleRate

            self._device_info = info

            if not self.isUniversalStream:
                await self.initDataTransfer(False)

            self._is_initing = False
            return True
        except Exception as e:
            self._is_initing = False
            raise RuntimeError("Init %s fail: %s" % (self._device_info.DeviceName, e))
            return False

    async def start_streaming(self) -> bool:
        if self._is_data_transfering:
            return True
        self._is_data_transfering = True
        self._rawDataBuffer.queue.clear()
        self._concatDataBuffer.clear()
        self.clear()

        if not self.isUniversalStream:
            await self.gForce.start_streaming(self._rawDataBuffer)
        else:
            await self.gForce.set_subscription(self.notifyDataFlag)

        return True

    async def stop_streaming(self) -> bool:
        if not self._is_data_transfering:
            return True

        self._is_data_transfering = False

        try:

            if not self.isUniversalStream:
                await self.gForce.stop_streaming()
            else:
                await self.gForce.set_subscription(0)

            while self._is_running and not self._rawDataBuffer.empty():
                await asyncio.sleep(0.1)

        except Exception as e:
            raise RuntimeError("Stop stream %s fail: %s" % (self._device_info.DeviceName, e))
            return False

        return True

    async def setFilter(self, filter: str, value: str) -> str:
        self.filter_map[filter] = value
        switch = 0
        for filter in self.filter_map.keys():
            value = self.filter_map[filter]
            if filter == "FILTER_50HZ":
                if value == "ON":
                    switch |= 1
            elif filter == "FILTER_60HZ":
                if value == "ON":
                    switch |= 2
            elif filter == "FILTER_HPF":
                if value == "ON":
                    switch |= 4
            elif filter == "FILTER_LPF":
                if value == "ON":
                    switch |= 8
        try:
            ret = await self.gForce.set_firmware_filter_switch(switch)
            if ret == None:
                return "ERROR: not success"
            return "OK"
        except Exception as e:
            return "ERROR: " + str(e)

    async def setDebugCSV(self, debugFilePath) -> str:
        if self.debugCSVWriter != None:
            self.debugCSVWriter = None
        if debugFilePath != None:
            self.debugCSVPath = debugFilePath
            try:
                if self.debugCSVPath != "":
                    CSVWriter = csv.writer(open(self.debugCSVPath, "w", newline=""), delimiter=",")
                    CSVWriter = None
            except Exception as e:
                return "ERROR: " + str(e)
        return "OK"

    ####################################################################################

    async def process_data(self, buf: Queue[SensorData], on_data_callback, on_error_callback=None):
        while self._is_running:
            while self._is_running and self._rawDataBuffer.empty():
                if self._is_running and self.isDataTransfering and not buf.empty():
                    sensorData: SensorData = None
                    try:
                        sensorData = buf.get_nowait()
                    except Exception as e:
                        break
                    if not sensor_utils._terminated and sensorData != None and on_data_callback != None:
                        try:
                            # on_data_callback(sensorData)
                            asyncio.get_event_loop().run_in_executor(self.dataPool, on_data_callback, sensorData)
                        except Exception as e:
                            pass

                    buf.task_done()
                else:
                    await asyncio.sleep(0.01)
                continue

            try:
                while self._is_running and not self._rawDataBuffer.empty():
                    data = self._rawDataBuffer.get_nowait()

                    if self.notifyDataFlag & DataSubscription.DNF_CONCAT_BLE != 0:
                        self._concatDataBuffer.extend(data)
                    else:
                        self._processDataPackage(data, buf, on_error_callback)

                    self._rawDataBuffer.task_done()
            except Exception as e:
                pass

            if self.notifyDataFlag & DataSubscription.DNF_CONCAT_BLE != 0:
                index = 0
                last_cut = -1
                data_size = len(self._concatDataBuffer)

                while self._is_running:
                    if index >= data_size:
                        break

                    if self._concatDataBuffer[index] == 0x55:
                        if (index + 1) >= data_size:
                            index = data_size
                            continue
                        n = self._concatDataBuffer[index + 1]
                        if n < 2 or (index + 1 + n + 1) >= data_size:
                            index += 1
                            continue
                        crc8 = (self._concatDataBuffer[index + 1 + n + 1])
                        calc_crc = sensor_utils.calc_crc8(self._concatDataBuffer[index + 2: index + 2 + n])
                        if crc8 != calc_crc:
                            index += 1
                            continue
                        if self._is_data_transfering:
                            data_package = bytes(self._concatDataBuffer[index + 2: index + 2 + n])
                            self._processDataPackage(data_package, buf, on_error_callback)
                        last_cut = index = index + 2 + n
                        index += 1
                    else:
                        index += 1

                if last_cut > 0:
                    self._concatDataBuffer = self._concatDataBuffer[last_cut + 1:]
                    last_cut = -1
                    index = 0

    def _processDataPackage(self, data: bytes, buf: Queue[SensorData], on_error_callback=None):
        v = data[0] & 0x7F
        if v == DataType.NTF_IMPEDANCE:
            offset = 1
            # packageIndex = ((data[offset + 1] & 0xff) << 8) | (data[offset] & 0xff)
            offset += 2

            impedanceData = []
            saturationData = []

            dataCount = (len(data) - 3) // 4 // 2

            for index in range(dataCount):
                impedance = struct.unpack_from("<f", data, offset)[0]
                offset += 4
                impedanceData.append(impedance)

            for index in range(dataCount):
                saturation = struct.unpack_from("<f", data, offset)[0]
                offset += 4
                saturationData.append(saturation / 10)  # firmware value range 0 - 1000

            self.impedanceData = impedanceData
            self.saturationData = saturationData
        elif v == DataType.NTF_IMPEDANCE_EXT:
            offset = 1
            # packageIndex = ((data[offset + 1] & 0xff) << 8) | (data[offset] & 0xff)
            offset += 2

            impedanceData = []
            saturationData = []

            dataCount = self._device_info.EegChannelCount + self._device_info.EcgChannelCount

            for index in range(dataCount):
                impedance = struct.unpack_from("<f", data, offset)[0]
                offset += 4
                impedanceData.append(impedance)

            for index in range(dataCount):
                saturation = struct.unpack_from("<H", data, offset)[0]
                offset += 2
                saturationData.append(saturation / 10)  # firmware value range 0 - 1000

            self.impedanceData = impedanceData
            self.saturationData = saturationData
        elif v == DataType.NTF_MAG_ANGLE_DATA:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_MAG_ANGLE]
            if self.checkReadSamples(data, sensor_data, 4, 0, on_error_callback):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_EMG:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_EMG]
            if self.checkReadSamples(data, sensor_data, sensor_data.packageIndexLength + 1, 0, on_error_callback):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_EEG:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_EEG]
            if self.checkReadSamples(data, sensor_data, 3, 0, on_error_callback):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_ECG:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_ECG]
            if self.checkReadSamples(data, sensor_data, 3, 0, on_error_callback):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_BRTH:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_BRTH]
            if self.checkReadSamples(data, sensor_data, 3, 0, on_error_callback):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_IMU:
            self._process_imu_samples(data, buf, on_error_callback)
        elif v == DataType.NTF_PPG:
            sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_PPG]
            if self.checkReadSamples(data, sensor_data, 3, 0, on_error_callback):
                self.sendSensorData(sensor_data, buf)
        elif v == DataType.NTF_SPO2:
            self._process_spo2_data(data, buf)
        elif v == DataType.NTF_EULER_DATA:
            self._process_gforce_float_data(data, buf, SensorDataType.DATA_TYPE_EULER, 3)
        elif v == DataType.NTF_QUATERNION:
            self._process_gforce_float_data(data, buf, SensorDataType.DATA_TYPE_GFORCE_QUAT, 4)
        elif v == DataType.NTF_ACC:
            self._process_gforce_int_data(data, buf, SensorDataType.DATA_TYPE_ACC, 3)
        elif v == DataType.NTF_GYRO:
            self._process_gforce_int_data(data, buf, SensorDataType.DATA_TYPE_GYRO, 3)

    def _process_gforce_float_data(self, data: bytes, buf: Queue[SensorData],
                                    data_type_index: int, channel_count: int):

        HEADER_SIZE = 2
        FLOAT_SIZE = 4

        EXPECTED_SIZE = HEADER_SIZE + channel_count * FLOAT_SIZE

        
        if len(data) != EXPECTED_SIZE:
            print(f"[DataTask] 异常: len(data){len(data)} ")
            return

        sensor_data = self.sensorDatas[data_type_index]


        if not sensor_data.channelSamples:
            sensor_data.channelSamples = [[] for _ in range(channel_count)]

        if sensor_data.lastPackageCounter < 0:
            sensor_data.lastPackageCounter = 0

        offset = HEADER_SIZE
        sample_idx = sensor_data.lastPackageCounter
        interval_ms = (1000 // sensor_data.sampleRate) if sensor_data.sampleRate > 0 else 0

        for ch in range(channel_count):
            raw_float = struct.unpack_from("<f", data, offset)[0]
            offset += FLOAT_SIZE
            val = raw_float if math.isfinite(raw_float) else 0.0

            sample = Sample()
            sample.channelIndex = ch
            sample.sampleIndex = sample_idx
            sample.timeStampInMs = sample_idx * interval_ms
            sample.rawData = raw_float
            sample.data = val
            sample.impedance = 0.0
            sample.saturation = 0.0
            sample.isLost = False
            sensor_data.channelSamples[ch].append(sample)

        sensor_data.lastPackageCounter += 1
        self.sendSensorData(sensor_data, buf)

    def _process_gforce_int_data(self, data: bytes, buf: Queue[SensorData],
                                  data_type_index: int, channel_count: int):

        HEADER_SIZE  = 1
        PKG_IDX_SIZE = 1
        INT_SIZE     = 4
        EXPECTED_SIZE = HEADER_SIZE + PKG_IDX_SIZE + channel_count * INT_SIZE

        if len(data) != EXPECTED_SIZE:
            print(f"[INT_DATA] type=0x{data[0]:02X} 包长不匹配 expected={EXPECTED_SIZE} actual={len(data)}")
            return

        sensor_data = self.sensorDatas[data_type_index]


        pkg_index = data[HEADER_SIZE] & 0xFF


        if not sensor_data.channelSamples:
            sensor_data.channelSamples = [[] for _ in range(channel_count)]


        if sensor_data.lastPackageCounter < 0:
            sensor_data.lastPackageCounter = 0
            sensor_data.lastPackageIndex = pkg_index
        else:
            if pkg_index == sensor_data.lastPackageIndex:
                return
        sensor_data.lastPackageIndex = pkg_index


        offset = HEADER_SIZE + PKG_IDX_SIZE
        sample_idx = sensor_data.lastPackageCounter
        interval_ms = (1000 // sensor_data.sampleRate) if sensor_data.sampleRate > 0 else 0

        channel_values = []
        for ch in range(channel_count):
            raw_int = struct.unpack_from("<i", data, offset)[0]
            offset += INT_SIZE
            channel_values.append(raw_int)

            sample = Sample()
            sample.channelIndex  = ch
            sample.sampleIndex   = sample_idx
            sample.timeStampInMs = sample_idx * interval_ms
            sample.rawData       = raw_int
            sample.data          = float(raw_int)
            sample.impedance     = 0.0
            sample.saturation    = 0.0
            sample.isLost        = False
            sensor_data.channelSamples[ch].append(sample)

        # type_name = "ACC" if data[0] & 0x7F == DataType.NTF_ACC else "GYRO"
        # print(f"[{type_name}] pkgIdx={pkg_index} x={channel_values[0]} y={channel_values[1]} z={channel_values[2]}")

        sensor_data.lastPackageCounter += 1
        self.sendSensorData(sensor_data, buf)

    def _process_spo2_data(self, data: bytes, buf: Queue[SensorData]):


        DATA_HEADER_SIZE = 3
        SPO2_CHANNEL_INDEX = 0
        HEART_RATE_CHANNEL_INDEX = 1
        REQUIRED_DATA_SIZE = 4

        offset = DATA_HEADER_SIZE

        if len(data) < offset + REQUIRED_DATA_SIZE:
            return


        spo2_value = struct.unpack_from(">H", data, offset)[0]
        offset += 2
        heart_rate = struct.unpack_from(">H", data, offset)[0]


        sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2]
        if not sensor_data.channelSamples:
            sensor_data.channelSamples = [[], []]


        self._append_spo2_sample(sensor_data, SPO2_CHANNEL_INDEX, spo2_value)
        self._append_spo2_sample(sensor_data, HEART_RATE_CHANNEL_INDEX, heart_rate)


        sensor_data.lastPackageCounter += 1
        self.sendSensorData(sensor_data, buf)

    def _append_spo2_sample(self, sensor_data: SensorData, channel_index: int, raw_value: int):

        sample = Sample()
        sample.channelIndex = channel_index
        sample.sampleIndex = sensor_data.lastPackageCounter
        sample.timeStampInMs = self._calculate_timestamp(sensor_data)
        sample.rawData = raw_value
        sample.data = float(raw_value)
        sample.impedance = 0.0
        sample.saturation = 0.0
        sample.isLost = False

        sensor_data.channelSamples[channel_index].append(sample)

    def _calculate_timestamp(self, sensor_data: SensorData) -> int:

        if sensor_data.sampleRate > 0:
            return sensor_data.lastPackageCounter * (1000 // sensor_data.sampleRate)
        return 0

    def _process_imu_samples(self, data: bytes, buf: Queue[SensorData], on_error_callback=None):
        sensor_data_acc = self.sensorDatas[SensorDataType.DATA_TYPE_ACC]
        sensor_data_gyro = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO]
        sensor_data_quat = None
        sensor_data_euler = None

        sensor_datas = [sensor_data_acc, sensor_data_gyro]
        if self.isContainQAT6:
            sensor_data_quat = self.sensorDatas[SensorDataType.DATA_TYPE_QUATERNION]
            sensor_data_euler = self.sensorDatas[SensorDataType.DATA_TYPE_EULER]
            sensor_datas.append(sensor_data_quat)
            sensor_datas.append(sensor_data_euler)

        if self._check_read_imu_samples(data, sensor_datas, sensor_data_quat, sensor_data_euler, on_error_callback):
            for sensor_data in sensor_datas:
                self.sendSensorData(sensor_data, buf)

    def _check_read_imu_samples(self, data: bytes, sensor_datas: List[SensorData], sensor_data_quat, sensor_data_euler,on_error_callback=None):
        if not self._is_data_transfering or not sensor_datas:
            return False

        try:
            sensor_data_ref = sensor_datas[0]
            offset = 1
            packageIndex = 0
            maxPackageIndex = 0

            if len(data) < offset + sensor_data_ref.packageIndexLength:
                return False

            if sensor_data_ref.packageIndexLength == 2:
                packageIndex = ((data[offset + 1] & 0xFF) << 8) | (data[offset] & 0xFF)
                maxPackageIndex = 65535
            elif sensor_data_ref.packageIndexLength == 1:
                packageIndex = data[offset] & 0xFF
                maxPackageIndex = 255

            offset += sensor_data_ref.packageIndexLength
            if not self._has_complete_imu_packet(data, sensor_data_ref, sensor_data_quat, offset):
                return False

            if sensor_data_ref.packageIndexLength <= 0:
                for sensor_data in sensor_datas:
                    if sensor_data.lastPackageCounter < 0:
                        sensor_data.lastPackageIndex = 0
                        sensor_data.lastPackageCounter = 0
            else:
                newPackageIndex = packageIndex
                lastPackageIndex = sensor_data_ref.lastPackageIndex

                if sensor_data_ref.lastPackageCounter < 0:
                    if newPackageIndex == 0:
                        lastPackageIndex = maxPackageIndex
                    else:
                        lastPackageIndex = newPackageIndex - 1

                    for sensor_data in sensor_datas:
                        sensor_data.lastPackageIndex = lastPackageIndex
                        sensor_data.lastPackageCounter = 0

                if packageIndex < lastPackageIndex:
                    packageIndex += maxPackageIndex + 1
                elif packageIndex == lastPackageIndex:
                    return False

                deltaPackageIndex = packageIndex - lastPackageIndex
                if deltaPackageIndex > 1:
                    lostSampleCount = sensor_data_ref.packageSampleCount * (deltaPackageIndex - 1)

                    if lostSampleCount < 100:
                        self._read_imu_frame_samples(data, sensor_datas, sensor_data_quat, 0, lostSampleCount)

                    if newPackageIndex == 0:
                        lastPackageIndex = maxPackageIndex
                    else:
                        lastPackageIndex = newPackageIndex - 1

                    for sensor_data in sensor_datas:
                        sensor_data.lastPackageIndex = lastPackageIndex
                        sensor_data.lastPackageCounter += deltaPackageIndex - 1

                    lostLog = (
                            "MSG|LOST SAMPLE|MAC|"
                            + str(sensor_data_ref.deviceMac)
                            + "|TYPE|"
                            + str(DataType.NTF_IMU)
                            + "|COUNT|"
                            + str(lostSampleCount)
                    )
                    if on_error_callback is not None:
                        try:
                            asyncio.get_event_loop().run_in_executor(None, on_error_callback, lostLog)
                        except Exception as e:
                            pass

                for sensor_data in sensor_datas:
                    sensor_data.lastPackageIndex = newPackageIndex

            if not self._read_imu_frame_samples(data, sensor_datas, sensor_data_quat, sensor_data_euler, offset, 0):
                return False

            for sensor_data in sensor_datas:
                sensor_data.lastPackageCounter += 1

            return True
        except Exception as e:
            return False

    def _has_complete_imu_packet(
            self,
            data: bytes,
            sensor_data_ref: SensorData,
            sensor_data_quat: SensorData,
            dataOffset: int,
    ) -> bool:
        frameSize = 12
        if sensor_data_quat is not None:
            frameSize += 12

        return len(data) >= dataOffset + sensor_data_ref.packageSampleCount * frameSize

    def _read_imu_frame_samples(
            self,
            data: bytes,
            sensor_datas: List[SensorData],
            sensor_data_quat: SensorData,
            sensor_data_euler: SensorData,
            dataOffset: int,
            lostSampleCount: int,
    ) -> bool:
        sensor_data_acc = sensor_datas[0]
        sensor_data_gyro = sensor_datas[1]
        sampleCount = sensor_data_acc.packageSampleCount
        if lostSampleCount > 0:
            sampleCount = lostSampleCount

        frameSize = 12
        if sensor_data_quat is not None:
            frameSize += 12

        if lostSampleCount <= 0 and len(data) < dataOffset + sampleCount * frameSize:
            return False

        for sensor_data in sensor_datas:
            if not sensor_data.channelSamples:
                sensor_data.channelSamples = [[] for _ in range(sensor_data.channelCount)]

        lastSampleIndex = sensor_data_acc.lastPackageCounter * sensor_data_acc.packageSampleCount
        offset = dataOffset

        for sampleOffset in range(sampleCount):
            sampleIndex = lastSampleIndex + sampleOffset

            if lostSampleCount > 0:
                self._append_imu_vector_sample(sensor_data_acc, sampleIndex, self.lastAccData, True)
                self._append_imu_vector_sample(sensor_data_gyro, sampleIndex, self.lastGyroData, True)
                if sensor_data_quat is not None:
                    self._append_imu_quaternion_sample(sensor_data_quat, sensor_data_euler, sampleIndex, self.lastQuatData, True)

                continue

            accRaw = [
                struct.unpack_from("<h", data, offset)[0],
                struct.unpack_from("<h", data, offset + 2)[0],
                struct.unpack_from("<h", data, offset + 4)[0],
            ]
            self.lastAccData = accRaw.copy()
            gyroOffset = offset + 6
            gyroRaw = [
                struct.unpack_from("<h", data, gyroOffset)[0],
                struct.unpack_from("<h", data, gyroOffset + 2)[0],
                struct.unpack_from("<h", data, gyroOffset + 4)[0],
            ]
            self.lastGyroData = gyroRaw.copy()

            self._append_imu_vector_sample(sensor_data_acc, sampleIndex, self.lastAccData, False)
            self._append_imu_vector_sample(sensor_data_gyro, sampleIndex, self.lastGyroData, False)

            if sensor_data_quat is not None:
                quatOffset = offset + 12
                quatRaw = [
                    struct.unpack_from("<i", data, quatOffset)[0],
                    struct.unpack_from("<i", data, quatOffset + 4)[0],
                    struct.unpack_from("<i", data, quatOffset + 8)[0],
                ]
                self._append_imu_quaternion_sample(sensor_data_quat, sensor_data_euler, sampleIndex, quatRaw, False)

            offset += frameSize

        return True

    def _append_imu_vector_sample(self, sensor_data: SensorData, sampleIndex: int, rawValues: List[int], isLost: bool):
        sampleInterval = 1000 // sensor_data.sampleRate if sensor_data.sampleRate > 0 else 0

        for channelIndex in range(sensor_data.channelCount):
            if (sensor_data.channelMask & (1 << channelIndex)) == 0:
                continue

            rawData = 0 if isLost else rawValues[channelIndex]

            sample = Sample()
            sample.channelIndex = channelIndex
            sample.sampleIndex = sampleIndex
            sample.timeStampInMs = sampleIndex * sampleInterval
            sample.rawData = rawData
            sample.data = 0.0 if isLost else rawData * sensor_data.K
            sample.impedance = 0.0
            sample.saturation = 0.0
            sample.isLost = isLost
            sensor_data.channelSamples[channelIndex].append(sample)

    def _append_imu_quaternion_sample(
            self,
            sensor_data_quat: SensorData,
            sensor_data_euler: SensorData,
            sampleIndex: int,
            rawValues: List[int],
            isLost: bool,
    ):
        scale = 1073741824.0  # 2^30
        sampleInterval = 1000 // sensor_data_quat.sampleRate if sensor_data_quat.sampleRate > 0 else 0

        raw = [0, 0, 0, 0]
        value = [0.0, 0.0, 0.0, 0.0]
        euler = [0.0, 0.0, 0.0]

        if not isLost:
            raw[1] = rawValues[0]
            raw[2] = rawValues[1]
            raw[3] = rawValues[2]
            
            value[1] = raw[1] / scale
            value[2] = raw[2] / scale
            value[3] = raw[3] / scale
            value[0] = math.sqrt(max(0.0, 1.0 - value[1] ** 2 - value[2] ** 2 - value[3] ** 2))

            euler[0] = math.atan2(2 * (value[0] * value[1] + value[2] * value[3]), 1 - 2 * (value[1] ** 2 + value[2] ** 2)) * 180 / math.pi
            euler[1] = math.asin(max(-1.0, min(1.0, 2 * (value[0] * value[2] - value[3] * value[1])))) * 180 / math.pi
            euler[2] = math.atan2(2 * (value[0] * value[3] + value[1] * value[2]), 1 - 2 * (value[2] ** 2 + value[3] ** 2)) * 180 / math.pi

            self.lastQuatData = value.copy()
            self.lastEulerData = euler.copy()

        for channelIndex in range(sensor_data_quat.channelCount):
            if (sensor_data_quat.channelMask & (1 << channelIndex)) == 0:
                continue

            sample = Sample()
            sample.channelIndex = channelIndex
            sample.sampleIndex = sampleIndex
            sample.timeStampInMs = sampleIndex * sampleInterval
            sample.rawData = raw[channelIndex]
            sample.data = self.lastQuatData[channelIndex]
            sample.impedance = 0.0
            sample.saturation = 0.0
            sample.isLost = isLost
            sensor_data_quat.channelSamples[channelIndex].append(sample)

        for channelIndex in range(sensor_data_euler.channelCount):
            if (sensor_data_euler.channelMask & (1 << channelIndex)) == 0:
                continue

            sample = Sample()
            sample.channelIndex = channelIndex
            sample.sampleIndex = sampleIndex
            sample.timeStampInMs = sampleIndex * sampleInterval
            sample.rawData = raw[channelIndex]
            sample.data = self.lastEulerData[channelIndex]
            sample.impedance = 0.0
            sample.saturation = 0.0
            sample.isLost = isLost
            sensor_data_euler.channelSamples[channelIndex].append(sample)

    def checkReadSamples(self, data: bytes, sensorData: SensorData, dataOffset: int, dataGap: int, on_error_callback=None):
        offset = 1

        if not self._is_data_transfering:
            return False
        try:
            packageIndex = 0
            maxPackageIndex = 0
            if (sensorData.packageIndexLength == 2):
                packageIndex = ((data[offset + 1] & 0xFF) << 8) | (data[offset] & 0xFF)
                maxPackageIndex = 65535
            elif (sensorData.packageIndexLength == 1):
                packageIndex = (data[offset] & 0xFF)
                maxPackageIndex = 255

            if sensorData.packageIndexLength <= 0:
                if sensorData.lastPackageCounter < 0:
                    sensorData.lastPackageIndex = 0
                    sensorData.lastPackageCounter = 0
            else:
                offset += sensorData.packageIndexLength
                newPackageIndex = packageIndex
                lastPackageIndex = sensorData.lastPackageIndex
                if sensorData.lastPackageCounter < 0 and newPackageIndex > 0:
                    sensorData.lastPackageIndex = lastPackageIndex = newPackageIndex - 1
                    sensorData.lastPackageCounter = 0

                if packageIndex < lastPackageIndex:
                    packageIndex += (maxPackageIndex + 1)
                elif packageIndex == lastPackageIndex:
                    return False

                deltaPackageIndex = packageIndex - lastPackageIndex
                lostPackageCounter = deltaPackageIndex - 1
                if lostPackageCounter > 65534:
                    lostPackageCounter = 1
                elif lostPackageCounter > 50:
                    lostPackageCounter = 50
                sensorData.lostPackageCount = sensorData.lostPackageCount + lostPackageCounter

                if deltaPackageIndex > 1:
                    lostSampleCount = sensorData.packageSampleCount * (deltaPackageIndex - 1)

                    if lostSampleCount < 100:
                        self.readSamples(data, sensorData, 0, dataGap, lostSampleCount)

                    if newPackageIndex == 0:
                        sensorData.lastPackageIndex = maxPackageIndex
                    else:
                        sensorData.lastPackageIndex = newPackageIndex - 1
                    sensorData.lastPackageCounter += deltaPackageIndex - 1

                    lostLog = (
                            "MSG|LOST SAMPLE|MAC|"
                            + str(sensorData.deviceMac)
                            + "|TYPE|"
                            + str(sensorData.dataType)
                            + "|COUNT|"
                            + str(lostSampleCount)
                    )
                    # print(lostLog)
                    if on_error_callback is not None:
                        try:
                            asyncio.get_event_loop().run_in_executor(None, on_error_callback, lostLog)
                        except Exception as e:
                            pass

                sensorData.lastPackageIndex = newPackageIndex

            if (dataGap >= 0):
                self.readSamples(data, sensorData, dataOffset, dataGap, 0)

            sensorData.lastPackageCounter += 1
        except Exception as e:
            # print(e)
            return False
        return True

    def transTrainData(self, data: int):
        xout = data >> 4
        exp = data & 0x0000000F
        xout = xout << exp
        return xout

    def readSamples(
            self,
            data: bytes,
            sensorData: SensorData,
            offset: int,
            dataGap: int,
            lostSampleCount: int,
    ):
        # 基本越界保护：offset 必须在 data 范围内
        if data is None or offset < 0 or offset > len(data):
            return

        sampleCount = sensorData.packageSampleCount
        sampleInterval = (
            1000 // sensorData.sampleRate if sensorData.sampleRate > 0 else 0
        )
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

        # 根据分辨率计算每个通道占用的字节数，用于后续越界检查
        if sensorData.resolutionBits in (7, 8):
            bytesPerChannel = 1
        elif sensorData.resolutionBits in (12, 16, 0):
            bytesPerChannel = 2
        elif sensorData.resolutionBits == 24:
            bytesPerChannel = 3
        else:
            bytesPerChannel = 2

        dataLength = len(data)

        for sampleIndex in range(sampleCount):
            for channelIndex, impedanceChannelIndex in enumerate(range(sensorData.channelCount)):
                if (sensorData.channelMask & (1 << channelIndex)) != 0:
                    samples = channelSamples[channelIndex]
                    impedance = 0.0
                    saturation = 0.0

                    if sensorData.dataType == DataType.NTF_ECG:
                        impedanceChannelIndex = self.sensorDatas[SensorDataType.DATA_TYPE_EEG].channelCount

                    if impedanceChannelIndex < len(_impedanceData):
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
                        # 在读取任何数据前先检查剩余字节是否足够
                        if offset + bytesPerChannel > dataLength:
                            return

                        rawData = 0
                        if sensorData.resolutionBits == 7:
                            rawData = data[offset]
                            rawData -= 119
                            offset += 1
                        elif sensorData.resolutionBits == 8:
                            rawData = data[offset] & 0xFF
                            offset += 1
                        elif sensorData.resolutionBits == 12:
                            rawData = int.from_bytes(
                                data[offset: offset + 2],
                                byteorder="little",
                                signed=True,
                            )
                            offset += 2
                        elif sensorData.resolutionBits == 16:
                            rawData = int.from_bytes(
                                data[offset: offset + 2],
                                byteorder="little",
                                signed=True,
                            )
                            offset += 2
                        elif sensorData.resolutionBits == 24:
                            rawData = (data[offset] << 16) | (data[offset + 1] << 8) | data[offset + 2]
                            if sensorData.dataType != DataType.NTF_PPG:
                                rawData -= 8388608
                            offset += 3
                        elif sensorData.resolutionBits == 0:
                            rawData = struct.unpack_from("<h", data, offset)[0]
                            offset += 2
                            rawData = self.transTrainData(rawData)

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
            sensorDataResult.lostPackageCount = sensorData.lostPackageCount
            sensorDataList.append(sensorDataResult)

            if self.debugCSVPath != None and self.debugCSVPath != "" and self.debugCSVWriter == None:
                try:
                    self.debugCSVWriter = csv.writer(open(self.debugCSVPath, "w", newline="", encoding="utf-8"))
                    header_append_keys = ["dataType", "sampleRate"]
                    channel_samples_header = list(vars(sensorDataResult.channelSamples[0][0]).keys())
                    for key_item in header_append_keys:
                        channel_samples_header.append(key_item)
                    self.debugCSVWriter.writerow(channel_samples_header)
                except Exception as e:
                    # print(e)
                    pass

            if self.debugCSVWriter != None:
                try:
                    for i, channel_sample_list in enumerate(sensorDataResult.channelSamples):
                        for channel_sample in channel_sample_list:
                            row_data = []

                            for key in vars(channel_sample).keys():
                                row_data.append(getattr(channel_sample, key))
                            row_data.append(sensorDataResult.dataType)
                            row_data.append(sensorDataResult.sampleRate)
                            self.debugCSVWriter.writerow(row_data)
                except Exception as e:
                    # print(e)
                    pass

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

    async def processUniversalData(self, buf: Queue[SensorData], on_data_callback, on_error_callback=None):

        while self._is_running:
            while self._is_running and self._rawDataBuffer.empty():
                if self._is_running and self.isDataTransfering and not buf.empty():
                    sensorData: SensorData = None
                    try:
                        sensorData = buf.get_nowait()
                    except Exception as e:
                        break
                    if not sensor_utils._terminated and sensorData != None and on_data_callback != None:
                        try:
                            # on_data_callback(sensorData)
                            asyncio.get_event_loop().run_in_executor(self.dataPool, on_data_callback, sensorData)
                        except Exception as e:
                            raise RuntimeError("Data callback %s fail: %s" % (self._device_info.DeviceName, e))

                    buf.task_done()
                else:
                    await asyncio.sleep(0.01)
                continue

            try:
                while self._is_running and not self._rawDataBuffer.empty():
                    data = self._rawDataBuffer.get_nowait()
                    self._concatDataBuffer.extend(data)
                    self._rawDataBuffer.task_done()
            except Exception as e:
                pass

            index = 0
            last_cut = -1
            data_size = len(self._concatDataBuffer)

            while self._is_running:
                if index >= data_size:
                    break

                if self._concatDataBuffer[index] == 0x55:
                    if (index + 1) >= data_size:
                        index = data_size
                        continue
                    n = self._concatDataBuffer[index + 1]
                    if n < 2 or (index + 1 + n + 2) >= data_size:
                        index += 1
                        continue
                    crc16 = (self._concatDataBuffer[index + 1 + n + 2] << 8) | self._concatDataBuffer[index + 1 + n + 1]
                    calc_crc = sensor_utils.crc16_cal(self._concatDataBuffer[index + 2: index + 2 + n], n)
                    if crc16 != calc_crc:
                        index += 1
                        continue
                    if self._is_data_transfering:
                        data_package = bytes(self._concatDataBuffer[index + 2: index + 2 + n])
                        self._processDataPackage(data_package, buf, on_error_callback)
                    last_cut = index = index + 2 + n + 1
                    index += 1
                elif self._concatDataBuffer[index] == 0xAA:
                    if (index + 1) >= data_size:
                        index = data_size
                        continue
                    n = self._concatDataBuffer[index + 1]
                    if n < 2 or (index + 1 + n + 2) >= data_size:
                        index += 1
                        continue
                    crc16 = (self._concatDataBuffer[index + 1 + n + 2] << 8) | self._concatDataBuffer[index + 1 + n + 1]
                    calc_crc = sensor_utils.crc16_cal(self._concatDataBuffer[index + 2: index + 2 + n], n)
                    if crc16 != calc_crc:
                        index += 1
                        continue
                    data_package = bytes(self._concatDataBuffer[index + 2: index + 2 + n])

                    if not sensor_utils._terminated:
                        await self.gForce.async_on_cmd_response(data_package)
                    last_cut = index = index + 2 + n + 1
                    index += 1
                else:
                    index += 1

            if last_cut > 0:
                self._concatDataBuffer = self._concatDataBuffer[last_cut + 1:]
                last_cut = -1
                index = 0
