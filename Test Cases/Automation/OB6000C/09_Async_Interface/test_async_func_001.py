# -*- coding: utf-8 -*-
"""ASYNC-FUNC-001：asyncScan 返回设备列表，与同步 scan 结果一致。

对应用例：09_异步接口.md -> ASYNC-FUNC-001
可自动化：auto（无需设备连接，仅需蓝牙开启且周围有任意 BLE 设备）

流程：
  1) 检查 SensorController.isEnable == True
  2) await ctrl.asyncScan(config.SCAN_TIMEOUT_MS) -> 断言返回 list
  3) ctrl.scan(config.SCAN_TIMEOUT_MS) -> 断言返回 list
  4) 比较两者设备数量一致
  5) 断言 async 结果中至少有一台设备 Name 和 Address 非空
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


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("ASYNC-FUNC-001 asyncScan 返回设备列表，与同步 scan 结果一致", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # ---- asyncScan ----
    print(f"\n[异步扫描] await ctrl.asyncScan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        devices_async = await ctrl.asyncScan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        devices_async = None
        print(f"[异步扫描] 抛异常 {type(e).__name__}: {e}", flush=True)

    is_list_async = isinstance(devices_async, list)
    print(f"[检查1] asyncScan 返回类型 = {type(devices_async).__name__}", flush=True)
    record(results, "asyncScan 返回 list[BLEDevice]", is_list_async,
           "返回类型为 list", f"返回类型 {type(devices_async).__name__}")

    if is_list_async:
        print(f"[异步扫描] 发现 {len(devices_async)} 台设备:", flush=True)
        for d in devices_async:
            name = getattr(d, 'Name', None) or '?'
            addr = getattr(d, 'Address', None) or '?'
            print(f"  - {name} {addr}", flush=True)

    # ---- 同步 scan ----
    print(f"\n[同步扫描] ctrl.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        devices_sync = ctrl.scan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        devices_sync = None
        print(f"[同步扫描] 抛异常 {type(e).__name__}: {e}", flush=True)

    is_list_sync = isinstance(devices_sync, list)
    print(f"[检查2] scan 返回类型 = {type(devices_sync).__name__}", flush=True)
    record(results, "scan 返回 list[BLEDevice]", is_list_sync,
           "返回类型为 list", f"返回类型 {type(devices_sync).__name__}")

    if is_list_sync:
        print(f"[同步扫描] 发现 {len(devices_sync)} 台设备:", flush=True)
        for d in devices_sync:
            name = getattr(d, 'Name', None) or '?'
            addr = getattr(d, 'Address', None) or '?'
            print(f"  - {name} {addr}", flush=True)

    # ---- 比较设备数量 ----
    if is_list_async and is_list_sync:
        count_async = len(devices_async)
        count_sync = len(devices_sync)
        count_match = count_async == count_sync
        print(f"\n[检查3] asyncScan 返回 {count_async} 台，scan 返回 {count_sync} 台，数量一致 = {count_match}", flush=True)
        record(results, "asyncScan 与 scan 返回设备数量一致", count_match,
               f"两者返回数量一致", f"asyncScan={count_async}, scan={count_sync}")
    else:
        record(results, "asyncScan 与 scan 返回设备数量一致", False,
               "两者返回数量一致", "至少一方返回非 list")

    # ---- 异步结果至少含一台有效设备 ----
    if is_list_async and len(devices_async) > 0:
        has_valid = False
        for d in devices_async:
            name = getattr(d, 'Name', None)
            addr = getattr(d, 'Address', None)
            if name and addr:
                has_valid = True
                print(f"\n[检查4] asyncScan 结果中含有效设备: Name={name}, Address={addr}", flush=True)
                break
        record(results, "asyncScan 结果至少含一台有效设备（Name/Address 非空）", has_valid,
               "至少一台设备 Name 和 Address 非空", f"有效设备={'是' if has_valid else '否'}")
    else:
        record(results, "asyncScan 结果至少含一台有效设备（Name/Address 非空）", False,
               "至少一台设备 Name 和 Address 非空", "asyncScan 返回空列表或非 list")

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
    asyncio.run(main_async())