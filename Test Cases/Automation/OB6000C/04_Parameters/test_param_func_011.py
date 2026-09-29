# -*- coding: utf-8 -*-
"""PARAM-FUNC-011：getParam IMU/PPG/EMG_SAMPLE_RATE 与 EMG_RESOLUTION（能力门控）。

对应用例：04_参数.md -> PARAM-FUNC-011
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getDeviceInfo() 读取能力，按 ChannelCount 判定各参数是否支持
  3) 逐项 getParam 查询（能力门控，不做硬编码）：
       IMU_SAMPLE_RATE / IMU_SAMPLE_RATE_LIST          -> 门控 ImuChannelCount
       PPG_SAMPLE_RATE / PPG_SAMPLE_RATE_LIST          -> 门控 PpgChannelCount
       EMG_SAMPLE_RATE / EMG_SAMPLE_RATE_LIST          -> 门控 EmgChannelCount
       EMG_RESOLUTION   / EMG_RESOLUTION_LIST          -> 门控 EmgChannelCount

判定口径（运行时能力，非硬编码）：
  - 对应 ChannelCount > 0（支持）：getParam 应返回非空、非 Error 的真实值。
  - 对应 ChannelCount == 0（不支持）：getParam 应返回以 Error 开头、空串，或抛异常；
    三者均为「不崩溃、正确表达不支持」，不判为缺陷。

OB6000C 预期（仅作说明，脚本不硬编码）：
  - ImuChannelCount=13 -> IMU 支持，IMU_SAMPLE_RATE 应为 "50"
  - PpgChannelCount=0  -> PPG 不支持，应返回 Error/空
  - EmgChannelCount=0  -> EMG / EMG_RESOLUTION 不支持，应返回 Error/空

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match


# (当前值 key, 列表 key, DeviceInfo 门控字段)
QUERY_SPECS = [
    ("IMU_SAMPLE_RATE", "IMU_SAMPLE_RATE_LIST", "ImuChannelCount"),
    ("PPG_SAMPLE_RATE", "PPG_SAMPLE_RATE_LIST", "PpgChannelCount"),
    ("EMG_SAMPLE_RATE", "EMG_SAMPLE_RATE_LIST", "EmgChannelCount"),
    ("EMG_RESOLUTION", "EMG_RESOLUTION_LIST", "EmgChannelCount"),
]


def _get(sensor, key):
    try:
        return sensor.getParam(key)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _check(sensor, info, key, field, results):
    """按 DeviceInfo 门控字段查询并记录一个 key 的结果。"""
    try:
        cnt = int(getattr(info, field, 0) or 0)
    except Exception as e:
        cnt = 0
        print(f"[能力] 读取 {field} 抛异常 {type(e).__name__}: {e}，按 0 处理", flush=True)
    supported = cnt > 0

    r = _get(sensor, key)
    print(f"[getParam] {key} -> {r!r}（{field}={cnt}，{'支持' if supported else '不支持'}）", flush=True)

    is_str = isinstance(r, str)
    is_error_or_empty = (not is_str) or (r == "") or r.startswith("Error") or ("异常" in r)
    is_real = is_str and r.strip() != "" and (not r.startswith("Error")) and ("异常" not in r)

    if supported:
        ok = is_real
        expect = f"{field}>0（支持）时 getParam({key}) 返回非空、非 Error 的真实值"
    else:
        ok = is_error_or_empty
        expect = f"{field}==0（不支持）时 getParam({key}) 返回 Error 开头/空串/抛异常（不崩溃）"

    record(results, f"getParam({key}) 能力门控", ok, expect, f"返回 {r!r}")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-011 getParam IMU/PPG/EMG_SAMPLE_RATE 与 EMG_RESOLUTION（能力门控）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

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

    # getDeviceInfo 用于能力门控
    info = sensor.getDeviceInfo()
    if info is None:
        print("[FAIL] getDeviceInfo() 返回 None，无法进行能力门控", flush=True)
        record(results, "getDeviceInfo() 返回 DeviceInfo", False,
               "getDeviceInfo() 返回 DeviceInfo（非 None）", "返回 None")
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "getDeviceInfo() 返回 DeviceInfo", True,
           "getDeviceInfo() 返回 DeviceInfo（非 None）", f"返回 {type(info).__name__}")

    # 逐项 getParam（能力门控）
    print("\n[参数] 逐项 getParam 查询（能力门控）...", flush=True)
    for value_key, list_key, field in QUERY_SPECS:
        _check(sensor, info, value_key, field, results)
        _check(sensor, info, list_key, field, results)

    # 清理
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
