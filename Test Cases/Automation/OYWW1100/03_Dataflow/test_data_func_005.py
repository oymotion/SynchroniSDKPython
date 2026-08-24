# -*- coding: utf-8 -*-
"""DATA-FUNC-005：各 DataType 流能收到对应 SensorData。

对应用例：03_数据流.md -> DATA-FUNC-005
可自动化：semi-auto（能力判定自动；需人工做手势/晃动/用力激发数据）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getDeviceInfo() 读取各模态 ChannelCount
  3) 对每个目标 DataType：
     - ChannelCount>0：setParam("NTF_xxx","ON") 起流，人工激发，统计 getDataType()
       断言收到该 DataType 数据且样本数>0
     - ChannelCount==0：记录"不支持，跳过"（SKIP）

目标模态（OYWW1100 腕带，据 probe_device_info 实测能力）与映射：
  DataType               setParam key        ChannelCount 字段    采集时长
  NTF_EMG                "NTF_EMG"           EmgChannelCount=8     5s
  NTF_GEST               "NTF_GEST"          EmgChannelCount=8     5s（DeviceInfo 无独立 GEST 标称值）
  NTF_IMPEDANCE          "NTF_IMPEDANCE"     ImpeChannelCount=8    25s（1Hz，需较长采集）
  NTF_ACC                "NTF_GFORCE_ACC"    AccChannelCount=3     5s
  NTF_GYRO               "NTF_GFORCE_GYRO"   GyroChannelCount=3    5s
  NTF_EULER_DATA         "NTF_GFORCE_EULER"  EulerChannelCount=3   5s
  NTF_QUATERNION         "NTF_GFORCE_QUAT"   QuatChannelCount=4    5s

说明：
  - GEST/EMG 在传统设备互斥，逐个测完 setParam OFF，避免串扰。
  - IMPEDANCE 采样率 1Hz，packageSampleCount=20 约需 20s 凑满一批，故采集 25s。
  - NTF_IMU 聚合流（ImuChannelCount=13）不在本条覆盖（见 DATA-FUNC-010）。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内，且已佩戴/可手持
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


# (显示名, DataType, setParam key, ChannelCount 字段名, 动作提示, 采集秒数)
MODALITIES = [
    ("EMG", DataType.NTF_EMG, "NTF_EMG", "EmgChannelCount",
     "请用力握拳或绷紧被测部位肌肉，让 EMG 产生信号", 5),
    ("GEST", DataType.NTF_GEST, "NTF_GEST", "EmgChannelCount",
     "请做手势动作（握拳/张手/翻转手腕等），触发手势识别", 5),
    ("IMPEDANCE", DataType.NTF_IMPEDANCE, "NTF_IMPEDANCE", "ImpeChannelCount",
     "请保持佩戴且电极接触良好，让阻抗测量产生信号", 25),
    ("ACC", DataType.NTF_ACC, "NTF_GFORCE_ACC", "AccChannelCount",
     "请晃动/移动腕带，让加速度计产生变化", 5),
    ("GYRO", DataType.NTF_GYRO, "NTF_GFORCE_GYRO", "GyroChannelCount",
     "请旋转/晃动腕带，让陀螺仪产生变化", 5),
    ("EULER", DataType.NTF_EULER_DATA, "NTF_GFORCE_EULER", "EulerChannelCount",
     "请旋转/晃动腕带，让欧拉角产生变化", 5),
    ("QUAT", DataType.NTF_QUATERNION, "NTF_GFORCE_QUAT", "QuatChannelCount",
     "请旋转/晃动腕带，让四元数产生变化", 5),
]

# DeviceInfo 候选字段（来自 README L313-323 与 example），用于运行时 dump 验证真实字段名
DEVICE_INFO_FIELDS = [
    "DeviceName", "ModelName", "HardwareVersion", "FirmwareVersion", "MTUSize",
    "PpgChannelCount", "PpgSampleRate",
    "Spo2ChannelCount", "Spo2SampleRate",
    "ImpeChannelCount", "ImpeSampleRate",
    "EmgChannelCount", "EmgSampleRate",
    "EegChannelCount", "EegSampleRate",
    "EcgChannelCount", "EcgSampleRate",
    "AccChannelCount", "AccSampleRate",
    "GyroChannelCount", "GyroSampleRate",
    "BrthChannelCount", "BrthSampleRate",
    "MagAngleChannelCount", "MagAngleSampleRate",
    "EulerChannelCount", "EulerSampleRate",
    "QuatChannelCount", "QuatSampleRate",
    "EmgMaxSampleRate", "EegMaxSampleRate", "EcgMaxSampleRate",
    "ImuChannelCount", "ImuSampleRate",
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs",
]


def dump_device_info(info):
    """运行时读取 DeviceInfo 全部候选字段，返回可读字符串（用于验证字段名是否正确）。"""
    parts = []
    for f in DEVICE_INFO_FIELDS:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"<{type(e).__name__}>"
        parts.append(f"{f}={v}")
    return ", ".join(parts)


class DataCollector:
    """按 DataType 统计收到的批次与样本数。"""

    def __init__(self):
        self.by_type = {}  # {DataType: {'batches': n, 'samples': m}}

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            entry = self.by_type.setdefault(dt, {'batches': 0, 'samples': 0})
            entry['batches'] += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            entry['samples'] += n

    def clear(self):
        self.by_type.clear()


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-005 各 DataType 流能收到对应 SensorData", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内，且已佩戴/可手持", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试中需要你配合做手势/晃动/用力动作，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含启用的目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含启用的目标设备", f"匹配到 {name} {addr}")

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # connect
    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True,
           "connect() 返回 True", f"connect() -> {connect_txt}")

    # 到达 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    ready = (sensor.deviceState == DeviceStateEx.Ready)
    record(results, "connect 后到达 Ready", ready, "deviceState==Ready", f"state={sensor.deviceState}")

    if not ready:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # init
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    # getDeviceInfo
    info = sensor.getDeviceInfo()
    record(results, "getDeviceInfo() 返回 DeviceInfo", info is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info).__name__ if info is not None else None}")

    if info is not None:
        print(f"\n[DeviceInfo 全字段 dump]", flush=True)
        print(f"  {dump_device_info(info)}", flush=True)

    if info is None:
        print("[FAIL] getDeviceInfo() 返回 None，无法进行能力判定", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 逐模态能力判定 + 数据验证
    collector = DataCollector()
    sensor.onDataCallback = collector.on_data

    # 先全部 OFF，清理初始状态（尤其 GEST/EMG 互斥）
    print("\n[准备] 关闭所有目标模态，清理初始状态 ...", flush=True)
    for disp, dt, param, field, hint, _sec in MODALITIES:
        try:
            sensor.setParam(param, "OFF")
        except Exception as e:
            print(f"  [清理] {param} OFF 抛异常 {type(e).__name__}: {e}", flush=True)

    for disp, dt, param, field, hint, collect_sec in MODALITIES:
        ch_count = getattr(info, field, 0)
        print(f"\n{'=' * 40}", flush=True)
        print(f"[模态] {disp}  DataType={_dt_name(dt)}  ChannelCount({field})={ch_count}", flush=True)
        print("=" * 40, flush=True)

        if ch_count <= 0:
            record(results, f"{disp} 流能收到数据（ChannelCount>0）", None,
                   f"ChannelCount>0 时收到 {_dt_name(dt)} 数据",
                   f"ChannelCount==0，设备不支持该模态")
            print(f"[SKIP] {disp} ChannelCount==0，设备不支持，跳过", flush=True)
            continue

        # setParam ON
        try:
            p_ret = sensor.setParam(param, "ON")
        except Exception as e:
            p_ret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] SensorProfile.setParam({param!r}, 'ON') -> {p_ret!r}", flush=True)
        record(results, f"{disp} setParam 返回 OK", p_ret == "OK",
               f"setParam({param!r}, 'ON') 返回 'OK'", f"返回 {p_ret!r}")

        # 起流
        try:
            s_ret = sensor.startDataNotification()
        except Exception as e:
            s_ret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[起流] startDataNotification() -> {s_ret!r}", flush=True)

        # 采集（人工激发）
        print(f"[动作] {hint}", flush=True)
        input("        按回车开始采集（采集期间请持续做动作）...")
        collector.clear()
        time.sleep(collect_sec)

        # 停流 + 关模态
        try:
            sensor.stopDataNotification()
        except Exception:
            pass
        try:
            sensor.setParam(param, "OFF")
        except Exception:
            pass

        # 统计该 DataType 数据
        entry = collector.by_type.get(dt, {'batches': 0, 'samples': 0})
        got = entry['samples'] > 0
        got_types = {_dt_name(k): v['batches'] for k, v in collector.by_type.items()}
        print(f"[采集] {disp} 收到批次={entry['batches']} 样本数={entry['samples']} 全部类型={got_types}", flush=True)
        record(results, f"{disp} 流能收到数据且类型匹配", got,
               f"收到 {_dt_name(dt)} 数据（样本数>0）",
               f"该类型样本数={entry['samples']} 批次={entry['batches']}")

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        elif status == "SKIP":
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status == "FAIL":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
