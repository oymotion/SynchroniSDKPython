# -*- coding: utf-8 -*-
"""CTRL-FUNC-002：蓝牙开启时 isEnable==True，onEnableCallback 触发。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-002
可自动化：semi-auto（需人工切换电脑蓝牙开关）

说明：本用例仅操作【电脑】蓝牙开关，无需待测设备 OB6000C 参与。

流程：
  1) 注册 SensorController.onEnableCallback
  2) 人工关闭电脑蓝牙 -> 按回车，等待回调
  3) 断言回调收到关闭事件(False)
  4) 人工开启电脑蓝牙 -> 按回车，等待回调
  5) 断言回调收到开启事件(True)，且 isEnable==True
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record


def wait_for_event(events, baseline, timeout=8.0):
    """等待 events 相对 baseline 出现新增（回调触发），最多等 timeout 秒。"""
    end = time.time() + timeout
    while time.time() < end:
        if len(events) > baseline:
            return True
        time.sleep(0.5)
    return len(events) > baseline


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-002 蓝牙开启时 isEnable==True，onEnableCallback 触发", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print("[说明] 本用例仅操作【电脑】蓝牙开关，无需待测设备 OB6000C 参与。", flush=True)

    # 注册使能回调
    enable_events = []

    def on_enable(enabled):
        enable_events.append(bool(enabled))
        print(f"  [SensorController.onEnableCallback] enabled={enabled}", flush=True)

    ctrl.onEnableCallback = on_enable
    print("[注册] SensorController.onEnableCallback 已就绪", flush=True)
    print(f"[初始] SensorController.isEnable = {ctrl.isEnable}", flush=True)

    results = []

    # ---- 阶段 1：关闭蓝牙 ----
    baseline1 = len(enable_events)
    input("\n>>> [人工操作] 请【关闭电脑】蓝牙（不是待测设备 OB6000C），完成后按回车继续 ...")
    wait_for_event(enable_events, baseline1)
    is_enable_off = ctrl.isEnable
    new_events1 = enable_events[baseline1:]
    off_txt = f"回调事件={[('开启' if e else '关闭') for e in new_events1]}"
    print(f"[检查1] 关闭蓝牙后 SensorController.isEnable = {is_enable_off}, {off_txt}", flush=True)
    got_off = any(e is False for e in new_events1)
    record(results, "关闭蓝牙后 SensorController.onEnableCallback 收到关闭事件(False)",
           got_off, "回调收到 enabled=False", off_txt)

    # ---- 阶段 2：开启蓝牙 ----
    baseline2 = len(enable_events)
    input("\n>>> [人工操作] 请【开启电脑】蓝牙（不是待测设备 OB6000C），完成后按回车继续 ...")
    wait_for_event(enable_events, baseline2)
    is_enable_on = ctrl.isEnable
    new_events2 = enable_events[baseline2:]
    on_txt = f"回调事件={[('开启' if e else '关闭') for e in new_events2]}"
    print(f"[检查2] 开启蓝牙后 SensorController.isEnable = {is_enable_on}, {on_txt}", flush=True)
    got_on = any(e is True for e in new_events2)
    record(results, "开启蓝牙后 SensorController.onEnableCallback 收到开启事件(True)",
           got_on, "回调收到 enabled=True", on_txt)

    record(results, "开启蓝牙后 SensorController.isEnable==True", is_enable_on is True,
           "SensorController.isEnable == True", f"SensorController.isEnable == {is_enable_on}")

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
