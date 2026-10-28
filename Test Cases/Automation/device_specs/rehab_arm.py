# -*- coding: utf-8 -*-
"""RehabArm 康复臂设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

同一型号（name_prefix == "RehabArm"）的多台设备共用本文件，
通过 config.MODEL_SPEC 映射、common.load_spec() 读取，无需逐次确认。

数据来源（实测固化，2026-10-27，设备 1D0E，probe_device.py 三层探测）：

DeviceInfo 声明支持的能力（ChannelCount>0）：
  - AccChannelCount=3   / AccSampleRate=50
  - AngleChannelCount=1 / AngleSampleRate=250    ← 角度（本型号核心能力）
  - EulerChannelCount=3 / EulerSampleRate=50
  - QuatChannelCount=4  / QuatSampleRate=50
  - ImuChannelCount=13  / ImuSampleRate=50
  - MagChannelCount=3   / MagSampleRate=10
  - GyroChannelCount 未声明（=0），但实际起流有 NTF_GYRO 数据（见下）

实际起流观测到的 DataType（probe 第 3 层）：
  - NTF_ACC(0x01):        3 通道, 50.0 Hz
  - NTF_GYRO(0x02):       3 通道, 50.0 Hz   （DeviceInfo 未声明 GyroChannelCount，设备固件不一致）
  - NTF_EULER_DATA(0x04): 3 通道, 50.0 Hz
  - NTF_QUATERNION(0x05): 4 通道, 50.0 Hz
  - NTF_ANGLE_DATA(0x0D): 1 通道, 250.0 Hz  ← 角度
  - NTF_MAG(0x03):        3 通道, 10.0 Hz   ← 磁力计

关键判定口径：
  - 无 NTF_IMU(0x13) 独立数据类型投递：IMU 数据按 acc/gyro/euler/quat 四路 DataType
    独立投递（与 gForceMax/gForceUltra 新固件一致），DeviceInfo.ImuChannelCount=13 是聚合声明。
  - 角度(1ch/250Hz) 与磁力计(3ch/10Hz) 为独立流，不并入 IMU 聚合（13 = 3+3+3+4）。
  - 纯运动/姿态设备：无 EMG/EEG/ECG/PPG/SpO2/呼吸/阻抗/手势（均 ChannelCount=0）。
  - 角度：单位 deg（度），量程约 [锐角值, 180°]：展开（伸直）≈180° 为上限，屈起≈某锐角值
    （<90°）为下限，即从 180° 向下递减；具体下限值/参考方向待与固件确认。采样率 250Hz 固定，不可设置。
  - 磁力计（Mag）：磁强计 3 轴（X/Y/Z），单位 uT（微特斯拉），10Hz。
  - 采样率参数档位：角度固定 250（settable=False，已确认不可改）；磁力计固定 10；
    IMU 是否可设档位本次 probe 未确认，暂按观测 50Hz 记录。
"""

SPEC = {
    # ===== 身份 =====
    "model": "RehabArm",
    "type": "康复臂/姿态追踪设备（3轴加速度 + 3轴陀螺仪 + 3轴欧拉角 + 4分量四元数 + 1通道角度 + 3轴磁力计）",
    "name_prefix": "RehabArm",

    # ===== 数据流能力（DataType 级别，对应 DeviceInfo 的 ChannelCount 期望）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # supported=False 表示该数据流不应存在（DeviceInfo 对应 ChannelCount==0）。
    # 注意：本型号以 DataType 枚举名（NTF_ACC/NTF_ANGLE_DATA 等）作为流键，
    #   与 gForceUltra/gForceMax 用参数 key（NTF_GFORCE_*）不同——因为 probe 观测到
    #   的实际 DataType 就是这些枚举名。
    "streams": {
        "NTF_ACC":          {"supported": True,  "channels": 3},   # 加速度 3 轴，50Hz
        "NTF_GYRO":         {"supported": True,  "channels": 3},   # 陀螺仪 3 轴，50Hz（DeviceInfo 未声明 GyroChannelCount，实测有数据）
        "NTF_EULER_DATA":   {"supported": True,  "channels": 3},   # 欧拉角 3 轴，50Hz
        "NTF_QUATERNION":   {"supported": True,  "channels": 4},   # 四元数 4 分量，50Hz
        "NTF_ANGLE_DATA":   {"supported": True,  "channels": 1},   # 角度 1 通道，250Hz 固定，单位 deg，量程约 [锐角值,180°]（展开180°、屈起为锐角）
        "NTF_MAG":          {"supported": True,  "channels": 3},   # 磁力计 3 轴（X/Y/Z），10Hz，单位 uT
        "NTF_IMU":          {"supported": True,  "channels": 13},  # 聚合声明 13=acc3+gyro3+euler3+quat4；实际按四路独立投递
        "NTF_EMG":          {"supported": False, "channels": 0},   # 无肌电
        "NTF_GEST":         {"supported": False, "channels": 0},   # 无手势
        "NTF_IMPEDANCE":    {"supported": False, "channels": 0},   # 无阻抗
        "NTF_EEG":          {"supported": False, "channels": 0},   # 无 EEG
        "NTF_ECG":          {"supported": False, "channels": 0},   # 无 ECG
        "NTF_BRTH":         {"supported": False, "channels": 0},   # 无呼吸
        "NTF_PPG":          {"supported": False, "channels": 0},   # 无 PPG
        "NTF_SPO2":         {"supported": False, "channels": 0},   # 无 SpO2
    },

    # ===== NTF_IMU 聚合流的子通道组成（无独立 NTF_IMU 数据类型，按四路独立投递）=====
    "imu_sub_channels": {
        "AccChannelCount":   3,   # 加速度 3 轴
        "GyroChannelCount":  3,   # 陀螺仪 3 轴（DeviceInfo.GyroChannelCount 未声明，实测有数据）
        "EulerChannelCount": 3,   # 欧拉角 3 轴
        "QuatChannelCount":  4,   # 四元数 4 分量
    },

    # ===== 滤波开关 =====
    # 纯运动/姿态设备，无生物电滤波参数（FILTER_*）；probe 未确认，暂空。
    "filters": {},

    # ===== 采样率（观测值；参数档位待 getParam 确认）=====
    # rates/default 为 probe 起流观测到的实际采样率。
    # settable 暂按 False（保守）：本次 probe 未固化 ANGLE/IMU/MAG_SAMPLE_RATE 的
    #   getParam/setParam 结论，待实测确认后可改为 True 并补档位。
    "sample_rates": {
        "IMU_SAMPLE_RATE": {
            "rates": ["50"], "default": "50",
            "stream": "NTF_IMU", "settable": False, "co_stream": None,
            # 无 NTF_IMU 独立数据类型：IMU 数据按四路 DataType 投递，
            # 采样率统计需聚合这四路（与 DeviceInfo.ImuChannelCount=13 一致）。
            "sub_streams": ["NTF_ACC", "NTF_GYRO", "NTF_EULER_DATA", "NTF_QUATERNION"],
        },
        "ANGLE_SAMPLE_RATE": {
            # 角度固定 250Hz、不可设置（demo 中角度无采样率选择，已确认 settable=False）
            "rates": ["250"], "default": "250",
            "stream": "NTF_ANGLE_DATA", "settable": False, "co_stream": None,
        },
        "MAG_SAMPLE_RATE": {
            # 磁力计 10Hz；是否可设置待确认，暂按固定记录
            "rates": ["10"], "default": "10",
            "stream": "NTF_MAG", "settable": False, "co_stream": None,
        },
    },
}
