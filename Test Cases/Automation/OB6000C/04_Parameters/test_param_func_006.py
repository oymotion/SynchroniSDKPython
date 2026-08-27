# -*- coding: utf-8 -*-
"""PARAM-FUNC-006：getParam EEG_SAMPLE_RATE/LIST。

对应用例：04_参数.md -> PARAM-FUNC-006
可自动化：auto

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getParam("EEG_SAMPLE_RATE") 应返回 "250"（OB6000C 固定 250）
  3) getParam("EEG_SAMPLE_RATE_LIST") 应返回 "250"（仅一个值）
  4) setParam("EEG_SAMPLE_RATE", "500") 应返回 Error（不支持）

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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-006 getParam EEG_SAMPLE_RATE/LIST", flush=True)
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

    # ---- getParam("EEG_SAMPLE_RATE")：OB6000C 固定 250 ----
    print("\n[getParam] EEG_SAMPLE_RATE ...", flush=True)
    try:
        current_rate = sensor.getParam("EEG_SAMPLE_RATE")
        print(f"[getParam] EEG_SAMPLE_RATE = {current_rate!r}", flush=True)
    except Exception as e:
        current_rate = f"抛异常 {type(e).__name__}: {e}"
        print(f"[getParam] EEG_SAMPLE_RATE 异常: {e}", flush=True)

    rate_is_250 = (isinstance(current_rate, str) and current_rate.strip() == "250")
    record(results, "getParam('EEG_SAMPLE_RATE') 返回 250",
           rate_is_250,
           "getParam('EEG_SAMPLE_RATE') == '250'（OB6000C 固定 250）",
           f"返回 {current_rate!r}")

    # ---- getParam("EEG_SAMPLE_RATE_LIST")：固定仅 250 ----
    print("\n[getParam] EEG_SAMPLE_RATE_LIST ...", flush=True)
    try:
        rate_list_str = sensor.getParam("EEG_SAMPLE_RATE_LIST")
        print(f"[getParam] EEG_SAMPLE_RATE_LIST = {rate_list_str!r}", flush=True)
    except Exception as e:
        rate_list_str = f"抛异常 {type(e).__name__}: {e}"
        print(f"[getParam] EEG_SAMPLE_RATE_LIST 异常: {e}", flush=True)

    list_is_250 = False
    if isinstance(rate_list_str, str):
        rates = [r.strip() for r in rate_list_str.split("|") if r.strip()]
        list_is_250 = (rates == ["250"])
        print(f"[getParam] EEG_SAMPLE_RATE_LIST 解析为: {rates}", flush=True)
    record(results, "getParam('EEG_SAMPLE_RATE_LIST') 返回仅 250",
           list_is_250,
           "getParam('EEG_SAMPLE_RATE_LIST') == '250'（OB6000C 固定 250）",
           f"返回 {rate_list_str!r}")

    # ---- setParam("EEG_SAMPLE_RATE", "500")：应报错 ----
    print("\n[setParam] EEG_SAMPLE_RATE='500'（不支持值）...", flush=True)
    try:
        r500 = sensor.setParam("EEG_SAMPLE_RATE", "500")
        ret500 = f"返回 {r500!r}"
    except Exception as e:
        r500 = f"抛异常 {type(e).__name__}: {e}"
        ret500 = f"抛异常 {type(e).__name__}: {e}"
    print(f"[setParam] EEG_SAMPLE_RATE='500' -> {ret500}", flush=True)

    is_error_500 = (isinstance(r500, str) and r500.startswith("Error")) or ("异常" in ret500)
    record(results, "setParam('EEG_SAMPLE_RATE','500') 返回 Error",
           is_error_500,
           "setParam('EEG_SAMPLE_RATE','500') 返回 Error 或抛异常",
           ret500)

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