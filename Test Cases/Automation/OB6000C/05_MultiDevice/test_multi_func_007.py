# -*- coding: utf-8 -*-
"""MULTI-FUNC-007：已起流设备再次 multiStart（restart 语义）。

对应用例：05_多设备同步.md -> MULTI-FUNC-007
可自动化：auto

流程：
  1) scan -> 匹配两台目标设备 -> connect -> init
  2) 第一台单独 startDataNotification 起流
  3) multiStartDataNotification([s1, s2])
  4) 验证 s1 先停后起，s2 起流，无异常

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
    print("MULTI-FUNC-007 已起流设备再次 multiStart（restart 语义）", flush=True)
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

    s1, s2 = sensors[0], sensors[1]
    tid1, tid2 = matched[0][1], matched[1][1]

    # 第一台单独起流
    print(f"\n[单独起流] {tid1} startDataNotification() ...", flush=True)
    try:
        sret = s1.startDataNotification()
        print(f"[单独起流] {tid1}: startDataNotification() -> {sret}, isDataTransfering={s1.isDataTransfering}", flush=True)
    except Exception as e:
        sret = None
        print(f"[单独起流] 抛异常 {type(e).__name__}: {e}", flush=True)

    if sret is not True:
        record(results, f"单独起流({tid1})", False,
               "startDataNotification() 返回 True", f"返回 {sret}")
        disconnect_all(sensors)
        print_summary(results, False)
        ctrl.terminate()
        return

    record(results, f"单独起流({tid1})", True,
           "startDataNotification() 返回 True", f"返回 {sret}")

    # 等待一下确保起流稳定
    time.sleep(2)

    print(f"\n[multiStart] 对已起流的 {tid1} 和未起流的 {tid2} 执行 multiStart ...", flush=True)
    try:
        mret = ctrl.multiStartDataNotification(sensors)
        print(f"[multiStart] 返回: {mret}", flush=True)
    except Exception as e:
        mret = None
        print(f"[multiStart] 抛异常 {type(e).__name__}: {e}", flush=True)

    restart_ok = False
    if isinstance(mret, dict):
        mac1 = s1.BLEDevice.Address
        mac2 = s2.BLEDevice.Address
        s1_ok = mret.get(mac1) is True
        s2_ok = mret.get(mac2) is True
        restart_ok = s1_ok and s2_ok
        print(f"[multiStart] {tid1}({mac1}): result={mret.get(mac1)}, isDataTransfering={s1.isDataTransfering}", flush=True)
        print(f"[multiStart] {tid2}({mac2}): result={mret.get(mac2)}, isDataTransfering={s2.isDataTransfering}", flush=True)

    record(results, "已起流设备 multiStart 先停后起（restart 语义）",
           restart_ok,
           "s1（已起流）先停后起，s2（空闲）起流，均成功，无异常",
           f"multiStart 返回 {mret}")

    try:
        ctrl.multiStopDataNotification(sensors)
    except Exception as e:
        print(f"[multiStop] 抛异常 {type(e).__name__}: {e}", flush=True)

    disconnect_all(sensors)
    print_summary(results, restart_ok)
    ctrl.terminate()


if __name__ == "__main__":
    main()