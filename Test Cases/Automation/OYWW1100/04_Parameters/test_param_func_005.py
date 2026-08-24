# -*- coding: utf-8 -*-
"""PARAM-FUNC-005：EEG_SAMPLE_RATE 不支持值返回 Error（待确认，按能力自动 SKIP）。

对应用例：04_参数.md -> PARAM-FUNC-005
可自动化：auto（能力判定后执行/跳过）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 能力判定：getParam("EEG_SAMPLE_RATE_LIST") 返回以 "Error" 开头 → SKIP
  3) 支持时：setParam("EEG_SAMPLE_RATE", 列表外值/空串)，校验返回以 "Error" 开头

说明：
  README：EEG_SAMPLE_RATE 值按能力列表校验，不支持值返回
  "Error: unsupported sample rate ..."。
  腕带 OYWW1100 通常无 EEG，预期 SKIP。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match

INVALID_RATES = ["999999", ""]  # 列表外值 + 空串


def _capability_ok(sensor):
    try:
        r = sensor.getParam("EEG_SAMPLE_RATE_LIST")
    except Exception as e:
        return False, f"抛异常 {type(e).__name__}: {e}"
    if not isinstance(r, str) or r.startswith("Error"):
        return False, r
    return True, r


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-005 EEG_SAMPLE_RATE 不支持值返回 Error", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
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

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

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

    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    # 能力判定
    supported, cap_txt = _capability_ok(sensor)
    print(f"\n[能力] getParam('EEG_SAMPLE_RATE_LIST') = {cap_txt!r}", flush=True)
    if not supported:
        record(results, "EEG_SAMPLE_RATE 不支持值返回 Error", None,
               "支持 EEG/ECG 采样率时校验非法值被拒绝",
               f"getParam('EEG_SAMPLE_RATE_LIST')={cap_txt!r}，设备未上报能力")
        print("[SKIP] 设备未上报 EEG/ECG 采样率能力，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # 逐个非法值测试
    for bad in INVALID_RATES:
        label = "空串" if bad == "" else f"列表外值{bad}"
        print(f"\n[参数] setParam('EEG_SAMPLE_RATE', {bad!r}) ...", flush=True)
        try:
            sret = sensor.setParam("EEG_SAMPLE_RATE", bad)
        except Exception as e:
            sret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[参数] setParam -> {sret!r}", flush=True)

        is_error = isinstance(sret, str) and sret.startswith("Error")
        record(results, f"EEG_SAMPLE_RATE 不支持值({label}) 返回 Error",
               is_error,
               "setParam 返回以 'Error' 开头",
               f"setParam->{sret!r}")

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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
