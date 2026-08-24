# -*- coding: utf-8 -*-
"""DEV-SM-001：connect 成功，状态迁移 Disconnected→Connecting→Connected→Ready。

对应用例：02_连接与状态机.md -> DEV-SM-001
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) 检查 SensorController.isEnable
  3) scan 匹配 OB6000C -> requireSensor
  4) 记录初始 deviceState（应 Disconnected）
  5) 注册 onStateChanged，记录状态迁移序列
  6) connect() -> 断言返回 True
  7) 断言最终 deviceState==Ready、isReady==True
  8) 断言状态迁移顺序 Connecting→Connected→Ready
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


def _idx(states, state_name):
    """返回 state_name 在 states 中首次出现的下标，不存在返回 None。"""
    try:
        return states.index(getattr(DeviceStateEx, state_name))
    except (ValueError, AttributeError):
        return None


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-001 connect 成功，状态迁移 Disconnected→Connecting→Connected→Ready", flush=True)
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

    # 扫描匹配
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备（OB6000C/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OB6000C", "未匹配到目标")
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OB6000C", f"匹配到 {name} {addr}")

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # 初始状态
    init_state = sensor.deviceState
    print(f"[检查1] connect 前 SensorProfile.deviceState = {init_state}", flush=True)
    record(results, "connect 前 deviceState==Disconnected", init_state == DeviceStateEx.Disconnected,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {init_state}")

    # 注册状态回调，记录迁移序列
    states = []

    def on_state(s, st):
        states.append(st)
        print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)

    sensor.onStateChanged = on_state

    # connect
    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = False
        print(f"[连接] SensorProfile.connect 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[连接] SensorProfile.connect() -> {ok}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True,
           "connect() 返回 True", f"connect() -> {ok}")

    # 最终状态
    final_state = sensor.deviceState
    is_ready = sensor.isReady
    print(f"[检查2] 连接后 SensorProfile.deviceState = {final_state}", flush=True)
    record(results, "连接后 deviceState==Ready", final_state == DeviceStateEx.Ready,
           "deviceState == DeviceStateEx.Ready", f"deviceState == {final_state}")

    print(f"[检查3] SensorProfile.isReady = {is_ready}", flush=True)
    record(results, "isReady==True", is_ready is True,
           "isReady == True", f"isReady == {is_ready}")

    # 状态迁移顺序
    print(f"[检查4] 状态迁移序列 = {[str(s) for s in states]}", flush=True)
    i_connecting = _idx(states, 'Connecting')
    i_connected = _idx(states, 'Connected')
    i_ready = _idx(states, 'Ready')

    if i_connected is not None and i_ready is not None and i_connected < i_ready:
        # 核心顺序：Connected 先于 Ready；Connecting 若出现应在 Connected 前
        if i_connecting is not None and i_connecting >= i_connected:
            order_ok = False
        else:
            order_ok = True
    else:
        order_ok = False

    record(results, "状态迁移顺序 Connecting→Connected→Ready", order_ok,
           "迁移顺序 Connecting→Connected→Ready（Connecting 可选）",
           f"序列 = {[str(s) for s in states]}")

    # disconnect 清理
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
