# -*- coding: utf-8 -*-
"""MULTI-FUNC-004：首包时差 ≤ maxDelayDispersionMs。

对应用例：05_多设备同步.md -> MULTI-FUNC-004
可自动化：auto

流程：
  1) scan -> 匹配两台目标设备 -> connect -> init
  2) multiStartDataNotification(sensors) 用默认参数（maxDelayDispersionMs=5, maxAttempts=3）
  3) 收集两台 startTimeStamp，计算散布
  4) 断言散布 ≤ 5ms（SDK 已自动重试至 maxAttempts）

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
    check_device_count, print_summary, MultiDataCollector,
)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MULTI-FUNC-004 首包时差 ≤ maxDelayDispersionMs", flush=True)
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

    # 挂载数据回调
    collector = MultiDataCollector()
    for s in sensors:
        s.onDataCallback = collector.on_data

    # multiStart（默认参数）
    print("\n[multiStart] multiStartDataNotification(sensors) 默认参数 (maxDelayDispersionMs=5, maxAttempts=3) ...", flush=True)
    try:
        mret = ctrl.multiStartDataNotification(sensors)
        print(f"[multiStart] 返回: {mret}", flush=True)
    except Exception as e:
        mret = None
        print(f"[multiStart] 抛异常 {type(e).__name__}: {e}", flush=True)

    started = isinstance(mret, dict) and all(v is True for v in mret.values())
    record(results, "multiStart 两台均起流", started,
           "两台 multiStart 返回均为 True", f"multiStart 返回 {mret}")

    if not started:
        disconnect_all(sensors)
        print_summary(results, False)
        ctrl.terminate()
        return

    # 等待数据到达
    print(f"\n[等待] 采集 {config.COLLECT_SECONDS}s 等待数据 ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)

    # 统计散布
    spread = collector.get_spread_ms()
    for s in sensors:
        mac = s.BLEDevice.Address
        fb = collector.first_batch.get(mac)
        if fb:
            print(f"[数据] {mac}: startTimeStamp={fb[0]}, delay={fb[1]}ms", flush=True)
        else:
            print(f"[数据] {mac}: 无数据", flush=True)

    if spread is not None:
        print(f"[散布] 跨设备 startTimeStamp 散布 = {spread} ms", flush=True)
        record(results, "跨设备 startTimeStamp 散布 ≤ 5ms",
               spread <= 5,
               f"散布 ≤ 5ms（默认 maxDelayDispersionMs=5）",
               f"散布 = {spread} ms")
    else:
        record(results, "跨设备 startTimeStamp 散布", False,
               "两台设备均收到带 startTimeStamp 的数据",
               f"数据不足: {collector.first_batch}")

    # 停流
    try:
        ctrl.multiStopDataNotification(sensors)
    except Exception as e:
        print(f"[multiStop] 抛异常 {type(e).__name__}: {e}", flush=True)

    disconnect_all(sensors)

    all_pass = started and (spread is not None and spread <= 5)
    print_summary(results, all_pass)
    ctrl.terminate()


if __name__ == "__main__":
    main()