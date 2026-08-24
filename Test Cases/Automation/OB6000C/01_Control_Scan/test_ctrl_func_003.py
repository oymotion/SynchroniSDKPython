# -*- coding: utf-8 -*-
"""CTRL-FUNC-003：startScan 成功返回 True，isScanning 翻转，并尝试连接设备以增强有效性。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-003
可自动化：semi-auto（需人工确认被测设备已开机）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) startScan -> 断言返回 True
  4) 断言 isScanning == True
  5) stopScan -> 断言 isScanning == False
  6) scan -> 断言发现目标设备 OB6000C
  7) requireSensor + connect -> 断言连接成功且到 Ready
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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-003 startScan 成功返回 True，isScanning 翻转 + 连接验证", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查：主机蓝牙必须已开启
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 检查1：startScan 返回 True
    try:
        ret = ctrl.startScan(config.SCAN_TIMEOUT_MS)
        ret_txt = f"返回 {ret}"
    except Exception as e:
        ret = None
        ret_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[检查1] SensorController.startScan({config.SCAN_TIMEOUT_MS}) -> {ret_txt}", flush=True)
    record(results, "SensorController.startScan 返回 True", ret is True,
           "SensorController.startScan 返回 True", ret_txt)

    # 检查2：startScan 后 isScanning == True
    is_scanning = ctrl.isScanning
    print(f"[检查2] startScan 后 SensorController.isScanning = {is_scanning}", flush=True)
    record(results, "startScan 后 SensorController.isScanning==True", is_scanning is True,
           "SensorController.isScanning == True", f"SensorController.isScanning == {is_scanning}")

    # stopScan（异常应计为 FAIL，而非仅警告）
    stop_raised = False
    stop_err = None
    try:
        ctrl.stopScan()
    except Exception as e:
        stop_raised = True
        stop_err = f"{type(e).__name__}: {e}"
        print(f"[检查] SensorController.stopScan 抛异常 {stop_err}", flush=True)
    record(results, "SensorController.stopScan 不抛异常", not stop_raised,
           "SensorController.stopScan 不抛异常", f"抛异常 {stop_err}" if stop_raised else "无异常")

    # 检查3：stopScan 后 isScanning == False
    time.sleep(1)
    is_scanning = ctrl.isScanning
    print(f"[检查3] stopScan 后 SensorController.isScanning = {is_scanning}", flush=True)
    record(results, "stopScan 后 SensorController.isScanning==False", is_scanning is False,
           "SensorController.isScanning == False", f"SensorController.isScanning == {is_scanning}")

    # ---- 增强：扫描发现设备并尝试连接 ----
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    n = len(devices) if devices else 0
    print(f"[扫描] 发现 {n} 台设备:", flush=True)
    for d in devices:
        print(f"  - {getattr(d, 'Name', '?')} {getattr(d, 'Address', '?')}", flush=True)

    if target is None:
        print("[检查4] 未匹配到 config 中启用的设备（OB6000C/80F3）", flush=True)
        record(results, "SensorController.scan 发现目标设备 OB6000C", False,
               "scan 返回含 OB6000C", f"发现 {n} 台，未匹配到目标")
    else:
        name = getattr(target, 'Name', '?')
        addr = getattr(target, 'Address', '?')
        print(f"[检查4] 匹配到目标设备: {name} {addr}", flush=True)
        record(results, "SensorController.scan 发现目标设备 OB6000C", True,
               "scan 返回含 OB6000C", f"匹配到 {name} {addr}")

        # 尝试连接
        sensor = ctrl.requireSensor(target)
        if sensor is None:
            print("[检查5] SensorController.requireSensor 返回 None", flush=True)
            record(results, "SensorController.requireSensor 返回 SensorProfile", False,
                   "requireSensor 返回非 None", "返回 None")
        else:
            sensor.onStateChanged = lambda s, st: print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)
            try:
                ok = sensor.connect()
            except Exception as e:
                ok = False
                print(f"[连接] SensorProfile.connect 抛异常 {type(e).__name__}: {e}", flush=True)
            state = sensor.deviceState
            print(f"[检查5] SensorProfile.connect() -> {ok}, deviceState = {state}", flush=True)
            record(results, "SensorProfile.connect 返回 True 且到 Ready", ok and state == DeviceStateEx.Ready,
                   "SensorProfile.connect 返回 True 且 deviceState==Ready",
                   f"connect 返回 {ok}, deviceState={state}")
            try:
                sensor.disconnect()
            except Exception as e:
                print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
