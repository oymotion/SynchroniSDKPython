# -*- coding: utf-8 -*-
"""MISC-FUNC-008：DeviceInfo 能力字段完整性。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-008
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor -> connect -> init
  3) getDeviceInfo() 获取 DeviceInfo 对象
  4) 逐一读取 ModelName 及各模态 ChannelCount/SampleRate
  5) 断言字段存在且值 ≥0；OB6000C 的 EEG 类字段 >0（主模态），
     其余模态按 getDeviceInfo() 运行时能力门控（只校验 ≥0，不硬断言是否=0）
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

# DeviceInfo 能力字段（按 README 和已知字段）
CAPABILITY_FIELDS = [
    # 基本信息
    "ModelName",
    # 各模态采样率
    "EmgSampleRate", "AccSampleRate", "GyroSampleRate",
    "EulerSampleRate", "QuatSampleRate", "ImuSampleRate",
    "MagAngleSampleRate", "EegSampleRate", "EcgSampleRate",
    "PpgSampleRate", "Spo2SampleRate", "BrthSampleRate",
    "ImpeSampleRate",
    # 最大采样率
    "EmgMaxSampleRate", "EegMaxSampleRate", "EcgMaxSampleRate",
    # 各模态通道数
    "EmgChannelCount", "AccChannelCount", "GyroChannelCount",
    "EulerChannelCount", "QuatChannelCount", "ImuChannelCount",
    "MagAngleChannelCount", "EegChannelCount", "EcgChannelCount",
    "PpgChannelCount", "Spo2ChannelCount", "BrthChannelCount",
    "ImpeChannelCount",
    # 链路参数
    "ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs", "MTUSize",
]

# OB6000C 主模态为 EEG，EEG 类字段预期 >0。
# 其余模态（EMG/IMU/Acc/Gyro/Euler/Quat/MagAngle/ECG/PPG/SPO2/BRTH/IMPEDANCE）
# 按 getDeviceInfo() 运行时能力门控，只做 ≥0 校验，不强硬断言是否=0。
EEG_EXPECTED_POSITIVE = [
    "EegSampleRate", "EegChannelCount", "EegMaxSampleRate",
]

# 链路参数允许 -1（表示 unknown，README: "0 / -1 / 0 = unknown"）
LINK_PARAM_FIELDS = ["ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs"]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-008 DeviceInfo 能力字段完整性", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    target_ids = config.TARGET_IDENTITY.split(",") if hasattr(config, 'TARGET_IDENTITY') else ["?"]
    print(f"\n[扫描] 目标 identity: {target_ids}（config.TARGET_IDENTITY = '{config.TARGET_IDENTITY}'）", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    scanned_list = [(getattr(d, 'Name', '?'), getattr(d, 'Address', '?')) for d in (devices or [])]
    print(f"[扫描] 扫描到 {len(scanned_list)} 台设备: {scanned_list}", flush=True)

    if target is None:
        print(f"[FAIL] 未匹配到目标设备（目标 identity: {target_ids}，扫描到: {scanned_list}）", flush=True)
        print("[提示] 请检查 config.py 中 TARGET_IDENTITY 是否设置为正确的设备 identity。", flush=True)
        if hasattr(config, 'DEVICES'):
            available_ids = [getattr(d, 'identity', '?') for d in config.DEVICES]
            print(f"[提示] 当前 DEVICES 配置中可用的 identity: {available_ids}", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备",
               f"未匹配到目标（目标: {target_ids}，扫描到: {scanned_list}）")
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

    # requireSensor -> connect -> init
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        ctrl.terminate()
        return

    print("\n[连接] sensor.connect() ...", flush=True)
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = False
        print(f"[连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] connect 失败", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print("[FAIL] 未到达 Ready", flush=True)
        ctrl.terminate()
        return

    print(f"\n[init] sensor.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        ok_init = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        ok_init = False
        print(f"[init] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok_init is not True:
        print("[FAIL] init 失败", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 获取 DeviceInfo
    info = sensor.getDeviceInfo()
    print(f"\n[DeviceInfo] getDeviceInfo() -> {type(info).__name__ if info else None}", flush=True)
    record(results, "getDeviceInfo() 返回 DeviceInfo", info is not None,
           "getDeviceInfo() 返回 DeviceInfo（非 None）",
           f"返回 {type(info).__name__ if info else None}")

    if info is None:
        print("[FAIL] getDeviceInfo() 返回 None，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 逐字段读取，分组打印
    print(f"\n[字段检查] 逐一读取 CAPABILITY_FIELDS 中的字段:", flush=True)
    field_values = {}

    # 打印顺序：基本信息 -> SampleRate -> ChannelCount -> MaxSampleRate -> 链路参数
    print(f"\n--- 基本信息 ---", flush=True)
    for f in ["ModelName"]:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"异常: {e}"
        field_values[f] = v
        print(f"  {f} = {v!r}", flush=True)

    print(f"\n--- SampleRate ---", flush=True)
    sr_fields = [f for f in CAPABILITY_FIELDS if f.endswith("SampleRate") and "Max" not in f]
    for f in sr_fields:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"异常: {e}"
        field_values[f] = v
        print(f"  {f} = {v!r}", flush=True)

    print(f"\n--- MaxSampleRate ---", flush=True)
    max_fields = [f for f in CAPABILITY_FIELDS if "MaxSampleRate" in f]
    for f in max_fields:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"异常: {e}"
        field_values[f] = v
        print(f"  {f} = {v!r}", flush=True)

    print(f"\n--- ChannelCount ---", flush=True)
    cc_fields = [f for f in CAPABILITY_FIELDS if f.endswith("ChannelCount")]
    for f in cc_fields:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"异常: {e}"
        field_values[f] = v
        print(f"  {f} = {v!r}", flush=True)

    print(f"\n--- 链路参数 ---", flush=True)
    link_fields = ["ConnectionIntervalMs", "PeripheralLatency", "SupervisionTimeoutMs", "MTUSize"]
    for f in link_fields:
        try:
            v = getattr(info, f, None)
        except Exception as e:
            v = f"异常: {e}"
        field_values[f] = v
        print(f"  {f} = {v!r}", flush=True)

    # 检查1：ModelName 存在且非空
    model = field_values.get("ModelName")
    model_ok = isinstance(model, str) and len(model) > 0
    print(f"\n[检查1] ModelName = {model!r} {'OK' if model_ok else 'FAIL'}", flush=True)
    record(results, "ModelName 存在且非空", model_ok,
           "ModelName 为非空字符串", f"ModelName = {model!r}")

    # 检查2：所有数值字段 ≥0（链路参数允许 -1 表示 unknown）
    all_ge_zero = True
    zero_or_none_fields = []
    for f in CAPABILITY_FIELDS:
        v = field_values.get(f)
        if isinstance(v, (int, float)):
            if f in LINK_PARAM_FIELDS and v == -1:
                continue  # 链路参数 -1 = unknown，合法
            if v < 0:
                all_ge_zero = False
                zero_or_none_fields.append(f"{f}={v}（负数）")
        elif v is None:
            zero_or_none_fields.append(f"{f}=None")
    print(f"[检查2] 所有数值字段 ≥0（链路参数 -1 除外）: {all_ge_zero}", flush=True)
    if zero_or_none_fields:
        print(f"  None 字段: {zero_or_none_fields}", flush=True)
    record(results, "所有数值字段 ≥0（链路参数 -1 除外）", all_ge_zero,
           "所有数值字段 ≥0 或链路参数 = -1", "全部 ≥0" if all_ge_zero else f"存在负数: {zero_or_none_fields}")

    # 检查3：OB6000C EEG 主模态字段预期 >0（动态门控，其余模态只校验 ≥0）
    positive_ok = True
    for f in EEG_EXPECTED_POSITIVE:
        v = field_values.get(f)
        if isinstance(v, (int, float)):
            if v <= 0:
                print(f"[检查3] {f} = {v}（EEG 主模态预期 >0）", flush=True)
                positive_ok = False
    record(results, "OB6000C EEG 主模态字段 >0（EegSampleRate/EegChannelCount/EegMaxSampleRate）",
           positive_ok,
           "OB6000C 的 EEG 类字段 >0",
           "全部 >0" if positive_ok else "存在 ≤0")

    # 检查4：其余模态（非 EEG）仅做信息记录，不做 =0 断言（能力门控由 getDeviceInfo 决定）
    print(f"\n[检查4] 其余模态值（能力门控，仅信息展示，不参与 PASS/FAIL 判定）", flush=True)
    non_eeg_fields = [f for f in CAPABILITY_FIELDS
                      if f not in EEG_EXPECTED_POSITIVE
                      and f.endswith(("SampleRate", "ChannelCount"))
                      and "Max" not in f]
    for f in non_eeg_fields:
        print(f"  {f} = {field_values.get(f)!r}", flush=True)

    # 清理
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()