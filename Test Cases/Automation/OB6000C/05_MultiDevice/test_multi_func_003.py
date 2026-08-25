# -*- coding: utf-8 -*-
"""MULTI-FUNC-003：某台未 Ready 不影响其余起流。

对应用例：05_多设备同步.md -> MULTI-FUNC-003
可自动化：auto

流程：
  1) scan -> 匹配两台目标设备
  2) 两台都 connect，但只有一台 init
  3) multiStartDataNotification([s1, s2])
  4) Ready 台起流，未 Ready 台失败，其余不受影响

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
    scan_and_match_all, disconnect_all,
    check_device_count, print_summary,
)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MULTI-FUNC-003 某台未 Ready 不影响其余起流", flush=True)
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

    # 连接两台设备
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

        sensors.append(sensor)

    # 只初始化第一台，第二台不初始化
    s1, s2 = sensors[0], sensors[1]
    tid1 = matched[0][1]
    tid2 = matched[1][1]

    print(f"\n[init] 只初始化 {tid1}，{tid2} 不初始化 ...", flush=True)
    try:
        iret = s1.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        iret = None
        print(f"[init] 抛异常 {type(e).__name__}: {e}", flush=True)

    record(results, f"init({tid1}) 返回 True", iret is True,
           "init() 返回 True", f"init() -> {iret}")

    if iret is not True:
        disconnect_all(sensors)
        print_summary(results, False)
        ctrl.terminate()
        return

    # multiStart
    print(f"\n[multiStart] multiStartDataNotification([{tid1}(已 init), {tid2}(未 init)]) ...", flush=True)
    try:
        mret = ctrl.multiStartDataNotification(sensors)
        print(f"[multiStart] 返回: {mret}", flush=True)
    except Exception as e:
        mret = None
        print(f"[multiStart] 抛异常 {type(e).__name__}: {e}", flush=True)

    ready_ok = False
    not_ready_ok = False
    if isinstance(mret, dict):
        mac1 = s1.BLEDevice.Address
        mac2 = s2.BLEDevice.Address
        ready_ok = (mret.get(mac1) is True and s1.isDataTransfering is True)
        not_ready_ok = (mret.get(mac2) is not True)
        print(f"[multiStart] {tid1}({mac1}): result={mret.get(mac1)}, isDataTransfering={s1.isDataTransfering}", flush=True)
        print(f"[multiStart] {tid2}({mac2}): result={mret.get(mac2)}, isDataTransfering={s2.isDataTransfering}", flush=True)

    record(results, f"已 Ready 设备({tid1})起流成功",
           ready_ok,
           "已 init 设备 multiStart 返回 True，isDataTransfering==True",
           f"已 init: {mret}")
    record(results, f"未 Ready 设备({tid2})起流失败，不影响其余",
           not_ready_ok,
           "未 init 设备 multiStart 返回 False，不影响已 init 设备",
           f"未 init: {mret}")

    # 停流
    try:
        ctrl.multiStopDataNotification(sensors)
    except Exception as e:
        print(f"[multiStop] 抛异常 {type(e).__name__}: {e}", flush=True)

    disconnect_all(sensors)
    all_pass = ready_ok and not_ready_ok
    print_summary(results, all_pass)
    ctrl.terminate()


if __name__ == "__main__":
    main()