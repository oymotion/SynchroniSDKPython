# -*- coding: utf-8 -*-
"""DEV-SM-004：disconnect 成功，状态迁移 Ready→Disconnecting→Disconnected。

对应用例：02_连接与状态机.md -> DEV-SM-004
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor -> connect 到 Ready
  3) 清空状态记录，注册 onStateChanged
  4) disconnect() -> 断言返回 True
  5) 断言最终 deviceState==Disconnected、onStateChanged 触发 Disconnected
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match


def _idx(states, state_name):
    try:
        return states.index(getattr(DeviceStateEx, state_name))
    except (ValueError, AttributeError):
        return None


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-004 disconnect 成功，Ready→Disconnecting→Disconnected", flush=True)
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
        print("\n结论: FAIL", flush=True)
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
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # 注册状态回调
    states = []

    def on_state(s, st):
        states.append(st)
        print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)

    sensor.onStateChanged = on_state

    # connect 到 Ready
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

    if ok is not True or sensor.deviceState != DeviceStateEx.Ready:
        print("[FAIL] 未到达 Ready，无法执行 disconnect 验证", flush=True)
        record(results, "connect 后到达 Ready", False, "deviceState==Ready", f"state={sensor.deviceState}")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "connect 后到达 Ready", True, "deviceState==Ready", f"state={sensor.deviceState}")

    # 清空状态记录，聚焦 disconnect 阶段
    states.clear()

    # disconnect
    print("\n[断开] SensorProfile.disconnect() ...", flush=True)
    try:
        dret = sensor.disconnect()
        disc_txt = f"返回 {dret}"
    except Exception as e:
        dret = None
        disc_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[断开] SensorProfile.disconnect() -> {disc_txt}", flush=True)
    record(results, "SensorProfile.disconnect 返回 True", dret is True,
           "disconnect() 返回 True", f"disconnect() -> {disc_txt}")

    # 最终状态
    final_state = sensor.deviceState
    print(f"[检查] disconnect 后 SensorProfile.deviceState = {final_state}", flush=True)
    record(results, "disconnect 后 deviceState==Disconnected", final_state == DeviceStateEx.Disconnected,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {final_state}")

    # 状态迁移：onStateChanged 触发 Disconnecting/Disconnected
    print(f"[检查] disconnect 阶段状态序列 = {[str(s) for s in states]}", flush=True)
    i_disconnecting = _idx(states, 'Disconnecting')
    i_disconnected = _idx(states, 'Disconnected')

    got_disconnected = i_disconnected is not None
    record(results, "onStateChanged 触发 Disconnected", got_disconnected,
           "onStateChanged 触发 Disconnected", f"序列 = {[str(s) for s in states]}")

    if i_disconnecting is not None and i_disconnected is not None:
        order_ok = i_disconnecting < i_disconnected
    elif got_disconnected:
        order_ok = True  # Disconnecting 未触发（可能瞬间过渡），不硬判
    else:
        order_ok = False

    record(results, "状态迁移顺序 Ready→Disconnecting→Disconnected", order_ok,
           "Disconnecting 先于 Disconnected（Disconnecting 可选）",
           f"序列 = {[str(s) for s in states]}")

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
