# -*- coding: utf-8 -*-
"""CTRL-FUNC-004：scan/asyncScan 返回 list[BLEDevice]，发现 OB6000C。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-004
可自动化：semi-auto（运行前需人工确认设备在范围内）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) scan -> 断言返回 list 且含 OB6000C 设备
  4) 断言 OB6000C 设备的 Address 非空、RSSI 非空
  5) asyncScan -> 断言返回 list 且含 OB6000C 设备
"""

import asyncio
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record


def _desc(d):
    name = getattr(d, 'Name', None) or '?'
    addr = getattr(d, 'Address', None) or '?'
    rssi = getattr(d, 'RSSI', None)
    return f"{name} {addr} RSSI={rssi}"


def _find_ob6000c(devices):
    """返回第一台 Name 以 OB6000C 开头的设备，无则 None。"""
    for d in devices:
        name = getattr(d, 'Name', None) or ''
        if name.upper().startswith('OB6000C'):
            return d
    return None


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-004 scan/asyncScan 返回 list[BLEDevice]，发现 OB6000C", flush=True)
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

    # ---- 同步 scan ----
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        devices = None
        print(f"[扫描] 抛异常 {type(e).__name__}: {e}", flush=True)

    is_list = isinstance(devices, list)
    print(f"[检查1] SensorController.scan 返回类型 = {type(devices).__name__}", flush=True)
    record(results, "SensorController.scan 返回 list[BLEDevice]", is_list,
           "返回类型为 list", f"返回类型 {type(devices).__name__}")

    if is_list:
        print(f"[扫描] 发现 {len(devices)} 台设备:", flush=True)
        for d in devices:
            print(f"  - {_desc(d)}", flush=True)
        target = _find_ob6000c(devices)
    else:
        target = None

    if target is None:
        print("[检查2] 未发现 Name 以 OB6000C 开头的设备", flush=True)
        record(results, "SensorController.scan 含 OB6000C 设备", False,
               "scan 结果含 Name 以 OB6000C 开头的设备", "未发现 OB6000C 设备")
    else:
        print(f"[检查2] 发现 OB6000C 设备: {_desc(target)}", flush=True)
        record(results, "SensorController.scan 含 OB6000C 设备", True,
               "scan 结果含 Name 以 OB6000C 开头的设备", f"发现 {getattr(target, 'Name', '?')}")

        addr = getattr(target, 'Address', None)
        print(f"[检查3] OB6000C 设备 Address = {addr!r}", flush=True)
        record(results, "OB6000C 设备 Address 非空", bool(addr),
               "Address 非空", f"Address={addr!r}")

        rssi = getattr(target, 'RSSI', None)
        print(f"[检查4] OB6000C 设备 RSSI = {rssi!r}", flush=True)
        record(results, "OB6000C 设备 RSSI 非空", rssi is not None,
               "RSSI 非空", f"RSSI={rssi!r}")

    # ---- asyncScan ----
    print(f"\n[扫描] SensorController.asyncScan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        devices_async = asyncio.run(ctrl.asyncScan(config.SCAN_TIMEOUT_MS))
    except Exception as e:
        devices_async = None
        print(f"[扫描] 抛异常 {type(e).__name__}: {e}", flush=True)

    is_list_async = isinstance(devices_async, list)
    has_ob6000c_async = bool(_find_ob6000c(devices_async)) if is_list_async else False
    print(f"[检查5] SensorController.asyncScan 返回类型 = {type(devices_async).__name__}, 含 OB6000C = {has_ob6000c_async}", flush=True)
    record(results, "SensorController.asyncScan 返回 list 且含 OB6000C 设备",
           is_list_async and has_ob6000c_async,
           "asyncScan 返回 list 且含 OB6000C 设备",
           f"返回类型 {type(devices_async).__name__}, 含 OB6000C={has_ob6000c_async}")

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
