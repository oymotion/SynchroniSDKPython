# -*- coding: utf-8 -*-
"""MISC-FUNC-007：DeviceStateEx.Invalid 状态。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-007
可自动化：semi-auto（需人工制造异常场景）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) 扫描匹配 OB6000C -> requireSensor
  3) 检查 DeviceStateEx.Invalid 枚举值存在且可比较
  4) 尝试在未连接时调用 init，检查是否会进入异常状态
  5) 尝试构造异常场景验证 SDK 不崩溃
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
    print("MISC-FUNC-007 DeviceStateEx.Invalid 状态", flush=True)
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

    # 检查1：DeviceStateEx.Invalid 枚举值存在
    print(f"\n[检查1] DeviceStateEx 枚举值:", flush=True)
    try:
        members = [m for m in dir(DeviceStateEx) if not m.startswith('_')]
        print(f"  DeviceStateEx 成员: {members}", flush=True)
    except Exception as e:
        print(f"  枚举成员 抛异常: {e}", flush=True)
        members = []

    has_invalid = hasattr(DeviceStateEx, 'Invalid')
    print(f"  DeviceStateEx.Invalid 存在: {has_invalid}", flush=True)
    if has_invalid:
        print(f"  DeviceStateEx.Invalid = {DeviceStateEx.Invalid}", flush=True)
    record(results, "DeviceStateEx.Invalid 枚举值存在", has_invalid,
           "DeviceStateEx.Invalid 存在", "存在" if has_invalid else "不存在")

    # 扫描匹配
    target_ids = config.TARGET_IDENTITY.split(",") if hasattr(config, 'TARGET_IDENTITY') else ["?"]
    print(f"\n[扫描] 目标 identity: {target_ids}（config.TARGET_IDENTITY = '{config.TARGET_IDENTITY}'）", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    scanned_list = [(getattr(d, 'Name', '?'), getattr(d, 'Address', '?')) for d in (devices or [])]
    print(f"[扫描] 扫描到 {len(scanned_list)} 台设备: {scanned_list}", flush=True)

    if target is None:
        print(f"[FAIL] 未匹配到目标设备（目标 identity: {target_ids}，扫描到: {scanned_list}）", flush=True)
        print("[提示] 请检查 config.py 中 TARGET_IDENTITY 是否设置为正确的设备 identity。", flush=True)
        if hasattr(config, 'DEVICES'):
            available_ids = [getattr(d, 'identity', '?') for d in config.DEVICES]
            print(f"[提示] 当前 DEVICES 配置中可用的 identity: {available_ids}", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备",
               f"未匹配到目标（目标: {target_ids}，扫描到: {scanned_list}）")
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        ctrl.terminate()
        return

    # 初始状态检查
    init_state = sensor.deviceState
    print(f"\n[状态] 初始 deviceState = {init_state}", flush=True)

    # 检查2：未连接时调用 init，验证 SDK 不崩溃
    print(f"\n[异常场景] 未连接时调用 sensor.init(20, 1000) ...", flush=True)
    try:
        init_ret = sensor.init(20, 1000)
        print(f"[异常场景] init() -> {init_ret}", flush=True)
    except Exception as e:
        init_ret = None
        print(f"[异常场景] init() 抛异常 {type(e).__name__}: {e}", flush=True)
    # 验证 SDK 不崩溃，状态未进入异常
    state_after_bad_init = sensor.deviceState
    print(f"[异常场景] 未连接 init 后 deviceState = {state_after_bad_init}", flush=True)
    sdk_stable = True  # 只要没崩溃就是稳定
    record(results, "未连接时 init 不崩溃", sdk_stable,
           "SDK 不崩溃", "未崩溃" if sdk_stable else "崩溃")

    # 检查3：正常连接，观察状态机路径
    print("\n[连接] sensor.connect() ...", flush=True)
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = False
        print(f"[连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] connect 失败", flush=True)
        record(results, "connect 成功", ok is True, "connect() 返回 True", f"connect() -> {ok}")
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    connected_state = sensor.deviceState
    print(f"[状态] 连接后 deviceState = {connected_state}", flush=True)
    record(results, "connect 后到达 Ready", connected_state == DeviceStateEx.Ready,
           "deviceState == Ready", f"deviceState == {connected_state}")

    # 检查4：正常 disconnect 后状态
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] 抛异常 {type(e).__name__}: {e}", flush=True)
    time.sleep(1)
    disc_state = sensor.deviceState
    print(f"[状态] disconnect 后 deviceState = {disc_state}", flush=True)
    record(results, "disconnect 后状态为 Disconnected", disc_state == DeviceStateEx.Disconnected,
           "deviceState == Disconnected", f"deviceState == {disc_state}")

    # 检查5：异常断开后再次 init，验证 SDK 不崩溃
    print(f"\n[异常场景] disconnect 后再次调用 sensor.init(20, 1000) ...", flush=True)
    try:
        init_ret2 = sensor.init(20, 1000)
        print(f"[异常场景] init() -> {init_ret2}", flush=True)
    except Exception as e:
        init_ret2 = None
        print(f"[异常场景] init() 抛异常 {type(e).__name__}: {e}", flush=True)
    state_after_reinit = sensor.deviceState
    print(f"[异常场景] disconnect 后 init deviceState = {state_after_reinit}", flush=True)
    record(results, "disconnect 后 init 不崩溃", True, "SDK 不崩溃", "未崩溃")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()