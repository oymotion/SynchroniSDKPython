# -*- coding: utf-8 -*-
"""MULTI-FUNC-006：混合型号放宽参数。

对应用例：05_多设备同步.md -> MULTI-FUNC-006
可自动化：auto（需同时有 OYWW1100 + OB6000C 两台设备）

流程：
  1) scan -> 匹配两台目标设备（不同型号）
  2) 通过 getDeviceInfo().ModelName 判断是否同型号
  3) 不同型号则用 timeout=60, maxDelayDispersionMs=-1, maxAttempts=5
  4) 断言成功起流

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 + OB6000C 各一台，上电、在范围内
  - config.TARGET_IDENTITY 需包含两台设备（如 "80F3,6C6B"）
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
    scan_and_match_all, disconnect_all,
    check_device_count, print_summary,
)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MULTI-FUNC-006 混合型号放宽参数", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"[本轮目标设备] {common.TARGET_IDENTITIES}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 + OB6000C 各一台，上电、在范围内", flush=True)
    print("  - config.TARGET_IDENTITY 需包含两台设备（如 '80F3,6C6B'）", flush=True)

    print("\n[配置检查]", flush=True)
    print(f"  TARGET_IDENTITY = {config.TARGET_IDENTITY}", flush=True)
    print(f"  目标设备: {len(common.TARGET_IDENTITIES)} 台", flush=True)
    for tid in common.TARGET_IDENTITIES:
        cfg = common._find_config(tid)
        if cfg:
            print(f"    - identity={tid}, name_prefix={cfg.get('name_prefix','?')}, mac={cfg.get('mac','') or '(auto)'}", flush=True)
    print("\n  请确认:", flush=True)
    print("  1. config.py 中 TARGET_IDENTITY 已正确配置（混合型号）", flush=True)
    print("  2. OYWW1100 + OB6000C 均已【开机】且在范围内", flush=True)

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

    # 连接所有设备
    sensors = []
    for tid, device in matched:
        sensor = ctrl.requireSensor(device)
        if sensor is None:
            record(results, f"requireSensor({tid})", False, "返回 SensorProfile", "返回 None")
            disconnect_all(sensors)
            print_summary(results, False)
            ctrl.terminate()
            return

        name = getattr(device, 'Name', '?')
        addr = getattr(device, 'Address', '?')
        print(f"\n[连接] {name} ({tid}) {addr} ...", flush=True)
        try:
            ok = sensor.connect()
        except Exception as e:
            ok = None
            print(f"[连接] 抛异常 {type(e).__name__}: {e}", flush=True)
        if ok is not True:
            record(results, f"connect({tid})", False, "connect() 返回 True", f"connect() -> {ok}")
            disconnect_all(sensors)
            print_summary(results, False)
            ctrl.terminate()
            return

        t0 = time.time()
        while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
            time.sleep(0.2)
        if sensor.deviceState != DeviceStateEx.Ready:
            record(results, f"到达 Ready({tid})", False, "deviceState==Ready", f"state={sensor.deviceState}")
            disconnect_all(sensors)
            print_summary(results, False)
            ctrl.terminate()
            return

        print(f"[init] {tid} ...", flush=True)
        try:
            iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        except Exception as e:
            iret = None
            print(f"[init] 抛异常 {type(e).__name__}: {e}", flush=True)
        if iret is not True:
            record(results, f"init({tid})", False, "init() 返回 True", f"init() -> {iret}")
            disconnect_all(sensors)
            print_summary(results, False)
            ctrl.terminate()
            return

        print(f"[就绪] {tid}: {name} {addr}", flush=True)
        sensors.append(sensor)

    # 打印各设备型号信息（仅供参考；前置条件已要求混合型号，由人工确认）
    for s, (_, tid) in zip(sensors, matched):
        info = s.getDeviceInfo()
        mn = info.ModelName if info else None
        print(f"[型号] {s.BLEDevice.Address} ({tid}): DeviceInfo.ModelName={mn}", flush=True)

    # 混合型号参数
    print("\n[multiStart] 混合型号: timeout=60, maxDelayDispersionMs=-1, maxAttempts=5 ...", flush=True)
    try:
        mret = ctrl.multiStartDataNotification(
            sensors, timeout=60.0, maxDelayDispersionMs=-1, maxAttempts=5)
        print(f"[multiStart] 返回: {mret}", flush=True)
    except Exception as e:
        mret = None
        print(f"[multiStart] 抛异常 {type(e).__name__}: {e}", flush=True)

    all_started = False
    if isinstance(mret, dict) and len(mret) == len(sensors):
        all_started = all(v is True for v in mret.values())
        for s in sensors:
            mac = s.BLEDevice.Address
            print(f"[multiStart] {mac}: result={mret.get(mac)}, isDataTransfering={s.isDataTransfering}", flush=True)

    record(results, "混合型号 multiStart 成功起流",
           all_started,
           "两台均起流（timeout=60, maxDelayDispersionMs=-1, maxAttempts=5）",
           f"multiStart 返回 {mret}")

    try:
        ctrl.multiStopDataNotification(sensors)
    except Exception as e:
        print(f"[multiStop] 抛异常 {type(e).__name__}: {e}", flush=True)

    disconnect_all(sensors)
    print_summary(results, all_started)
    ctrl.terminate()


if __name__ == "__main__":
    main()