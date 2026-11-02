# -*- coding: utf-8 -*-
"""OB3000 设备规格（Spec）——由 probe_device_info.py 自动生成的骨架。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

自动生成：能力字段已按 DeviceInfo + getParam + 起流实测填充；
标注 TODO 处需人工确认（co_stream / sub_streams / 互斥关系、单位/量程等）。

数据来源（probe 实测）：
  - 广播名='OB3000(C2D5)' MAC='F0:74:BF:AE:C2:D5'
  - DeviceInfo: ModelName='OB3000' Firmware='V1.0.33_2026-06-24_d0625ee' Hardware='0.1'
  - DeviceInfo 声明支持(ChannelCount>0): EegChannelCount=23, EcgChannelCount=1, ImpeChannelCount=24, AccChannelCount=3, GyroChannelCount=3, EulerChannelCount=3, QuatChannelCount=4, ImuChannelCount=13
  - 起流实测 DataType: NTF_ACC(ch=3, rate~50Hz), NTF_ECG(ch=1, rate~250Hz), NTF_EEG(ch=23, rate~250Hz), NTF_EULER_DATA(ch=3, rate~50Hz), NTF_GYRO(ch=3, rate~50Hz), NTF_IMPEDANCE(ch=24, rate~250Hz), NTF_QUATERNION(ch=4, rate~50Hz)
  - 采样率参数: EEG_SAMPLE_RATE=['250', '500']（当前 '250'）; EMG_SAMPLE_RATE='0'; IMU_SAMPLE_RATE=['50']（当前 '50'）; PPG_SAMPLE_RATE='0'
"""

SPEC = {
    # ===== 身份 =====
    "model": 'OB3000',
    "type": '3ch NTF_ACC；3ch NTF_GYRO；3ch NTF_EULER_DATA；4ch NTF_QUATERNION；23ch NTF_EEG；1ch NTF_ECG；24ch NTF_IMPEDANCE；13ch NTF_IMU',
    "name_prefix": 'OB3000',

    # ===== 数据流能力（DataType 级别，对应 DeviceInfo 的 ChannelCount）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # 通道数口径：优先起流实测值，其次 DeviceInfo 声明值（二者不一致处标 TODO）。
    "streams": {
        "NTF_ACC": {"supported": True, "channels": 3},  # DeviceInfo=3 实测=3
        "NTF_GYRO": {"supported": True, "channels": 3},  # DeviceInfo=3 实测=3
        "NTF_MAG": {"supported": False, "channels": 0},  # DeviceInfo=0
        "NTF_EULER_DATA": {"supported": True, "channels": 3},  # DeviceInfo=3 实测=3
        "NTF_QUATERNION": {"supported": True, "channels": 4},  # DeviceInfo=4 实测=4
        "NTF_GEST": {"supported": False, "channels": 0},  # DeviceInfo=0
        "NTF_EMG": {"supported": False, "channels": 0},  # DeviceInfo=0
        "NTF_ANGLE_DATA": {"supported": False, "channels": 0},  # DeviceInfo=0
        "NTF_EEG": {"supported": True, "channels": 23},  # DeviceInfo=23 实测=23
        "NTF_ECG": {"supported": True, "channels": 1},  # DeviceInfo=1 实测=1
        "NTF_IMPEDANCE": {"supported": True, "channels": 24},  # DeviceInfo=24 实测=24
        "NTF_IMU": {"supported": True, "channels": 13},  # DeviceInfo=13
        "NTF_BRTH": {"supported": False, "channels": 0},  # DeviceInfo=0
        "NTF_SPO2": {"supported": False, "channels": 0},  # DeviceInfo=0
        "NTF_PPG": {"supported": False, "channels": 0},  # DeviceInfo=0
    },

    # ===== NTF_IMU 聚合流的子通道组成（无独立 NTF_IMU 数据类型时按多路投递）=====
    "imu_sub_channels": {'AccChannelCount': 3, 'GyroChannelCount': 3, 'EulerChannelCount': 3, 'QuatChannelCount': 4},

    # ===== 滤波开关（FILTER_* 支持 ON/OFF）=====
    "filters": {
        "FILTER_50HZ": True,
        "FILTER_60HZ": True,
        "FILTER_HPF": True,
        "FILTER_LPF": True,
    },

    # ===== 采样率（供 test_measure_sample_rate.py 遍历）=====
    # stream    : setParam("NTF_XXX","ON") 开关 key，同时是 DataType.NTF_XXX 类型名
    # settable  : True 表示 key 即 setParam 采样率参数名；False 为固定采样率
    # co_stream : 伴随流（如 IMU/阻抗需与主数据流同起才有数据），None 表示无
    "sample_rates": {
        "EEG_SAMPLE_RATE": {
            "rates": ['250', '500'], "default": "250",
            "stream": "NTF_EEG", "settable": True, "co_stream": None,
        },  # settable；co_stream/sub_streams TODO
        "EMG_SAMPLE_RATE": {
            "rates": ['0'], "default": "0",
            "stream": "NTF_EMG", "settable": False, "co_stream": None,
        },  # 固定(观测值)；co_stream/sub_streams TODO
        "IMU_SAMPLE_RATE": {
            "rates": ['50'], "default": "50",
            "stream": "NTF_IMU", "settable": False, "co_stream": None,
            "sub_streams": ['NTF_ACC', 'NTF_GYRO', 'NTF_EULER_DATA', 'NTF_QUATERNION'],
        },  # 固定(观测值)；IMU 按 ['NTF_ACC', 'NTF_GYRO', 'NTF_EULER_DATA', 'NTF_QUATERNION'] 拆分投递 (TODO 确认)
        "PPG_SAMPLE_RATE": {
            "rates": ['0'], "default": "0",
            "stream": "NTF_PPG", "settable": False, "co_stream": None,
        },  # 固定(观测值)；co_stream/sub_streams TODO
        "IMPEDANCE": {
            "rates": ["1"], "default": "1",
            "stream": "NTF_IMPEDANCE", "settable": False, "co_stream": None,
        },  # 固定 1Hz；co_stream TODO
        "ECG_SAMPLE_RATE": {
            "rates": ["250"], "default": "250",
            "stream": "NTF_ECG", "settable": False, "co_stream": None,
        },  # 无采样率参数，按实测固定 (TODO 确认)
    },
}
