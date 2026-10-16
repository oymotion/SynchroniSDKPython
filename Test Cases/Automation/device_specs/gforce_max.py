# -*- coding: utf-8 -*-
"""gForceMax 腕带设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

同一型号（name_prefix == "gForceMax"）的多台设备共用本文件，
通过 config.MODEL_SPEC 映射、common.load_spec() 读取，无需逐次确认。

数据来源（实测固化，2026-10-14，设备 9A91，probe_device_info.py dump）：
  - DeviceName = 'gForceMax(9A91)'；ModelName = 'gForceUltra'（固件仍上报 gForceUltra，
    仅广播名区分 gForceMax，故 spec 加载靠 name_prefix，不依赖 ModelName）
  - Hardware 1.41，Firmware V1.0.1_2026-10-14_ce27cad
  - EmgChannelCount = 8；ImpeChannelCount = 8（8 电极接触检测）
  - ImuChannelCount = 13 = acc(3) + gyro(3) + euler(3) + quat(4)
  - AccChannelCount=3 / GyroChannelCount=3 / EulerChannelCount=3 / QuatChannelCount=4
  - EMG_SAMPLE_RATE_LIST = 500|1000，上电默认 1000（EmgMaxSampleRate=1000）
  - IMU_SAMPLE_RATE_LIST = 50|100|200（三档，gForceUltra 仅 50 一档）

关键差异（相对 gForceUltra）：
  - 无 NTF_GFORCE_ACC/GYRO/EULER/QUAT 独立开关（getParam 全部 Error: Not supported），
    acc/gyro/euler/quat 统一打包进 NTF_IMU 聚合流（13 通道，单一开关）。
  - 新增欧拉角（EulerChannelCount=3）与四元数（QuatChannelCount=4），gForceUltra 均为 0。
  - IMU 采样率三档 50/100/200（gForceUltra 固定 50）。

关键判定口径：
  getParam('NTF_XXX') 对部分 key 返回 'ON'，但 'ON' 只表示参数开关被设备接受，
  不等于该流有数据。本文件以 DeviceInfo 的 ChannelCount 为准：ChannelCount > 0 才算支持。
"""

SPEC = {
    # ===== 身份 =====
    "model": "gForceMax",
    "type": "EMG 腕带（8 通道肌电 + 手势识别 + 13 通道 IMU[acc3+gyro3+euler3+quat4] + 8 通道电极接触检测）",
    "name_prefix": "gForceMax",

    # ===== 数据流能力（对应 DeviceInfo 的 ChannelCount 期望）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # supported=False 表示该数据流不应存在（DeviceInfo 对应 ChannelCount==0，
    #   或该流无独立开关、被打包进其它聚合流）。
    # 注意：gForceMax 的 NTF_GFORCE_* 无独立开关（getParam 返回 Error），
    #   acc/gyro/euler/quat 能力见 imu_sub_channels，统一由 NTF_IMU 控制。
    "streams": {
        "NTF_EMG":          {"supported": True,  "channels": 8},   # 8 通道干电极肌电
        "NTF_GEST":         {"supported": True,  "channels": 1},   # 手势 ID（单值流）
        "NTF_GFORCE_ACC":   {"supported": False, "channels": 0},   # 无独立开关（acc 打包进 NTF_IMU）
        "NTF_GFORCE_GYRO":  {"supported": False, "channels": 0},   # 无独立开关（gyro 打包进 NTF_IMU）
        "NTF_GFORCE_EULER": {"supported": False, "channels": 0},   # 无独立开关（euler 打包进 NTF_IMU）
        "NTF_GFORCE_QUAT":  {"supported": False, "channels": 0},   # 无独立开关（quat 打包进 NTF_IMU）
        "NTF_IMU":          {"supported": True,  "channels": 13},  # ImuChannelCount=13（acc3+gyro3+euler3+quat4 聚合）
        "NTF_MAG_ANGLE":    {"supported": False, "channels": 0},   # 无磁力计
        "NTF_IMPEDANCE":    {"supported": True,  "channels": 8},   # ImpeChannelCount=8（电极接触检测，固定 1Hz，见 sample_rates.IMPEDANCE）
        "NTF_EEG":          {"supported": False, "channels": 0},   # 腕带无 EEG
        "NTF_ECG":          {"supported": False, "channels": 0},   # 腕带无 ECG
        "NTF_BRTH":         {"supported": False, "channels": 0},   # 腕带无呼吸
        "NTF_PPG":          {"supported": False, "channels": 0},   # 腕带无 PPG（NTF_PPG_RAW 为其别名）
        "NTF_SPO2":         {"supported": False, "channels": 0},   # 腕带无 SpO2
    },

    # ===== NTF_IMU 聚合流的子通道组成（无独立开关，统一由 NTF_IMU 控制）=====
    # 这是 gForceMax 相对 gForceUltra 的核心差异：IMU 从 6 轴扩到 13 轴
    # （新增欧拉角 3 轴 + 四元数 4 分量），但只有一个 NTF_IMU 开关。
    "imu_sub_channels": {
        "AccChannelCount":   3,   # 加速度 3 轴
        "GyroChannelCount":  3,   # 陀螺仪 3 轴
        "EulerChannelCount": 3,   # 欧拉角 3 轴（gForceMax 新增）
        "QuatChannelCount":  4,   # 四元数 4 分量（gForceMax 新增）
    },

    # ===== 滤波开关（FILTER_* 支持 ON/OFF）=====
    "filters": {
        "FILTER_50HZ": True,   # 50Hz 工频陷波
        "FILTER_60HZ": True,   # 60Hz 工频陷波
        "FILTER_HPF":  True,   # 10Hz 高通
        "FILTER_LPF":  True,   # 200Hz 低通
    },

    # ===== 采样率（统一事实来源，供 test_measure_sample_rate.py 遍历）=====
    # 每条对应一路数据流：
    #   stream    : setParam("NTF_XXX", "ON") 开关 key，同时是 DataType.NTF_XXX 类型名
    #   settable  : True 表示 key 即 setParam 采样率参数名；False 为固定采样率、无对应参数
    #   co_stream : 伴随流（需与其它流同起才有数据），None 表示可单独起流取数据
    "sample_rates": {
        "EMG_SAMPLE_RATE": {
            "rates": ["500", "1000"], "default": "1000",
            "stream": "NTF_EMG", "settable": True, "co_stream": None,
        },
        "IMU_SAMPLE_RATE": {
            "rates": ["50", "100", "200"], "default": "50",   # 默认待确认（probe 观察到当前 getParam=200，可能非上电默认）
            "stream": "NTF_IMU", "settable": True, "co_stream": None,
        },
        "IMPEDANCE": {
            "rates": ["1"], "default": "1",
            "stream": "NTF_IMPEDANCE", "settable": False, "co_stream": "NTF_EMG",
        },
    },
}
