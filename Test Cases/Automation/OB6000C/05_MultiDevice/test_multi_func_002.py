# -*- coding: utf-8 -*-
"""MULTI-FUNC-002：两台同时停流。

对应用例：05_多设备同步.md -> MULTI-FUNC-002
可自动化：auto

流程：
  1) scan -> 匹配两台目标设备 -> connect -> init
  2) multiStartDataNotification 起流
  3) multiStopDataNotification([s1, s2]) 停流
  4) 检查两台上流状态均为 False

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：≥2 台 OB6000C 上电、在范围内
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
from common import record
from multi_common import (
    scan_and_match_all, connect_and_init_all, disconnect_all,
    check_device_count, print_summary,
)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MULTI-FUNC-002 两台同时停流", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"[本轮目标设备] {common.TARGET_IDENTITIES}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：≥2 台 OB6000C 上电、在范围内", flush=True)

    print("\n[配置检查]", flush=True)
    print(f"  TARGET_IDENTITY = {config.TARGET_IDENTITY}", flush=True)
    print(f"  目标设备: {len(common.TARGET_IDENTITIES)} 台", flush=True)
    for tid in common.TARGET_IDENTITIES:
        cfg = common._find_config(tid)
        if cfg:
            print(f"    - identity={tid}, name_prefix={cfg.get('name_prefix','?')}, mac={cfg.get('mac','') or '(auto)'}", flush=True)
    print("\n  请确认:", flush=True)
    print("  1. config.py 中 TARGET_IDENTITY 已正确配置", flush=True)
    print("  2. 所有目标设备已【开机】且在范围内", flush=True)

    input("\n>>> [人工操作] 确认以上无误后，按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    matched, devices = scan_and_match_all(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, required=2)
    print(f"[匹配] 目标设备数: {len(common.TARGET_IDENTITIES)}, 匹配到: {len(matched)}", flush=True)
    if not check_device_count(results, matched, required=2):
        print_summary(results, True)
        ctrl.terminate()
        return

    sensors = connect_and_init_all(ctrl, matched, results)
    if len(sensors) < 2:
        disconnect_all(sensors)
        print_summary(results, False)
        ctrl.terminate()
        return

    # 先起流
    print("\n[multiStart] 起流 ...", flush=True)
    try:
        mret = ctrl.multiStartDataNotification(sensors)
        print(f"[multiStart] 返回: {mret}", flush=True)
    except Exception as e:
        mret = None
        print(f"[multiStart] 抛异常 {type(e).__name__}: {e}", flush=True)

    started = isinstance(mret, dict) and all(v is True for v in mret.values())
    if not started:
        record(results, "multiStart 起流", False, "两台均起流", f"multiStart 返回 {mret}")
        disconnect_all(sensors)
        print_summary(results, False)
        ctrl.terminate()
        return

    # multiStop
    print("\n[multiStop] multiStopDataNotification(...) ...", flush=True)
    try:
        sret = ctrl.multiStopDataNotification(sensors)
        print(f"[multiStop] 返回: {sret}", flush=True)
    except Exception as e:
        sret = None
        print(f"[multiStop] 抛异常 {type(e).__name__}: {e}", flush=True)

    all_stopped = False
    if isinstance(sret, dict) and len(sret) == len(sensors):
        all_stopped = all(v is True for v in sret.values())
        for s in sensors:
            mac = s.BLEDevice.Address
            print(f"[multiStop] {mac}: result={sret.get(mac)}, isDataTransfering={s.isDataTransfering}", flush=True)

    record(results, "multiStopDataNotification 两台均停流",
           all_stopped,
           "两台 multiStop 返回均为 True，isDataTransfering==False",
           f"multiStop 返回 {sret}")

    disconnect_all(sensors)
    print_summary(results, all_stopped)
    ctrl.terminate()


if __name__ == "__main__":
    main()