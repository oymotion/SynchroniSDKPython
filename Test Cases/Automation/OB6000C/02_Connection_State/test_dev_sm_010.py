# -*- coding: utf-8 -*-
"""DEV-SM-010：onAutoReconnect(restore=True) 自定义恢复（返回 True 跳过默认恢复）。

对应用例：02_连接与状态机.md -> DEV-SM-010
可自动化：semi-auto（需人工断电/移出范围，再恢复供电/移回范围）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan -> requireSensor -> 检查 autoReconnect==True（默认）
  3) 注册 onAutoReconnect 返回 True（表示由应用接管，跳过 SDK 默认恢复流程）
  4) connect 到 Ready -> init -> startDataNotification 起流
  5) 人工断电/移出范围 -> 观察断链（Disconnected + onErrorCallback）
  6) 人工恢复供电/移回范围 -> 等待 onAutoReconnect 触发（restore 参数）
     返回 True 后观察窗口内确认【跳过默认恢复】（未自动回到 Ready/未自动起流）

语义说明（来自 examples/SynchroniSDKPython_DemoNewMulti.py）：
  - onAutoReconnect(sensor, restore) 在自动重连找回设备时触发
  - 返回 True：由应用接管，SDK 不再执行默认的参数回放恢复
  - 返回 False：SDK 执行默认恢复（connect -> init -> replay setParam -> startDataNotification）
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config

DISCONNECT_TIMEOUT = 60     # 断链检测超时（秒）
RECONNECT_TIMEOUT = 120     # 等待 onAutoReconnect 触发超时（秒）
NO_RECONNECT_WINDOW = 30    # 观察窗口（秒），确认跳过默认恢复

from common import record, scan_and_match


def _wait_until(cond, timeout, interval=0.5, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(interval)
    print(f"  [等待超时] {what}（{timeout}s）", flush=True)
    return False


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-010 onAutoReconnect(restore=True) 自定义恢复", flush=True)
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

    # autoReconnect 默认值（本 case 依赖 autoReconnect=True）
    auto_reconnect = sensor.autoReconnect
    print(f"[检查] SensorProfile.autoReconnect = {auto_reconnect}", flush=True)
    record(results, "autoReconnect 默认为 True", auto_reconnect is True,
           "autoReconnect == True", f"autoReconnect == {auto_reconnect}")

    # 注册回调
    states = []
    errors = []
    reconnects = []

    def on_state(s, st):
        states.append(st)
        print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)

    def on_error(s, reason):
        errors.append(reason)
        print(f"  [SensorProfile.onErrorCallback] {s.BLEDevice.Name}: {reason!r}", flush=True)

    def on_reconnect(s, restore):
        reconnects.append(restore)
        print(f"  [SensorProfile.onAutoReconnect] {s.BLEDevice.Name} restore={restore!r}", flush=True)
        return True  # 自定义恢复接管，跳过 SDK 默认恢复流程

    sensor.onStateChanged = on_state
    sensor.onErrorCallback = on_error
    sensor.onAutoReconnect = on_reconnect

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

    _wait_until(lambda: sensor.deviceState == DeviceStateEx.Ready, 10, 0.2, "连接后等待 Ready")
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

    # init + 起流
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    print("[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    transferring = sensor.isDataTransfering
    record(results, "起流后 isDataTransfering==True", transferring is True,
           "isDataTransfering == True", f"isDataTransfering == {transferring}")

    if sret is not True:
        print("[FAIL] 起流失败，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # ---- 阶段 1：人工断电/移出范围，观察断链 ----
    input("\n>>> [人工操作] 请【关闭】待测设备 OB6000C 电源（或移出蓝牙范围），完成后按回车继续 ...")

    print(f"\n[等待断链] 最多 {DISCONNECT_TIMEOUT}s ...", flush=True)
    state_disconnected = _wait_until(
        lambda: sensor.deviceState == DeviceStateEx.Disconnected,
        DISCONNECT_TIMEOUT, 0.5, "deviceState==Disconnected")
    error_reported = len(errors) > 0
    print(f"[断链] deviceState={sensor.deviceState} 错误回调 {len(errors)} 次 errors={errors!r} 状态序列 {[str(s) for s in states]}", flush=True)

    record(results, "断链状态变化（deviceState==Disconnected）", state_disconnected,
           "deviceState==Disconnected",
           f"state={sensor.deviceState} 状态序列={[str(s) for s in states]}")
    record(results, "onErrorCallback 触发（异常断链原因上报）", error_reported,
           "onErrorCallback 至少触发 1 次并上报断链原因",
           f"errors={errors!r}（触发 {len(errors)} 次）")

    # ---- 阶段 2：人工恢复供电，观察 onAutoReconnect 自定义恢复 ----
    input("\n>>> [人工操作] 请【重新开启】待测设备 OB6000C 电源（或移回范围），完成后按回车继续 ...")

    print(f"\n[等待 onAutoReconnect] 最多 {RECONNECT_TIMEOUT}s ...", flush=True)
    cb_triggered = _wait_until(lambda: len(reconnects) > 0, RECONNECT_TIMEOUT, 0.5, "onAutoReconnect 触发")
    print(f"[回调] onAutoReconnect 触发 {len(reconnects)} 次 restore={reconnects!r}", flush=True)
    record(results, "onAutoReconnect 触发（restore 参数传入）", cb_triggered,
           "onAutoReconnect 至少触发 1 次",
           f"触发 {len(reconnects)} 次 restore={reconnects!r}")

    if cb_triggered:
        # 返回 True 后，观察窗口内确认跳过默认恢复
        print(f"\n[观察跳过默认恢复] 观察窗口 {NO_RECONNECT_WINDOW}s ...", flush=True)
        reached_ready = False
        t0 = time.time()
        while time.time() - t0 < NO_RECONNECT_WINDOW:
            if sensor.deviceState == DeviceStateEx.Ready:
                reached_ready = True
                print("  [观察] 检测到 deviceState==Ready（默认恢复未跳过），提前结束观察", flush=True)
                break
            time.sleep(0.5)

        transferring_after = sensor.isDataTransfering
        print(f"[观察] deviceState={sensor.deviceState} isDataTransfering={transferring_after} 状态序列 {[str(s) for s in states]}", flush=True)

        record(results, "返回 True 后跳过默认恢复（未自动回到 Ready）", not reached_ready,
               "onAutoReconnect 返回 True 后 deviceState 不自动回到 Ready",
               f"state={sensor.deviceState} 状态序列={[str(s) for s in states]}")
        record(results, "返回 True 后未自动起流（isDataTransfering 保持 False）", transferring_after is not True,
               "onAutoReconnect 返回 True 后 isDataTransfering 不自动恢复 True",
               f"isDataTransfering={transferring_after}")
    else:
        record(results, "返回 True 后跳过默认恢复（未自动回到 Ready）", None,
               "onAutoReconnect 返回 True 后 deviceState 不自动回到 Ready",
               "onAutoReconnect 未触发，无法验证（前置失败）")
        record(results, "返回 True 后未自动起流（isDataTransfering 保持 False）", None,
               "onAutoReconnect 返回 True 后 isDataTransfering 不自动恢复 True",
               "onAutoReconnect 未触发，无法验证（前置失败）")

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
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
