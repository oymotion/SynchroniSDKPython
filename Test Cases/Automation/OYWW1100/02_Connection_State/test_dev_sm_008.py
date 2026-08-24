# -*- coding: utf-8 -*-
"""DEV-SM-008：autoReconnect=True（默认）异常断链自动恢复。

对应用例：02_连接与状态机.md -> DEV-SM-008
可自动化：semi-auto（需人工断电/移出范围，再恢复供电/移回范围）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan -> requireSensor -> 检查 autoReconnect==True
  3) connect 到 Ready -> init -> startDataNotification 起流
  4) 人工断电/移出范围 -> 按回车 -> 观察断链可观测（Disconnected/onErrorCallback）
  5) 人工恢复供电/移回范围 -> 按回车 -> 观察自动重连恢复（onAutoReconnect -> 重新 Ready -> 恢复起流）
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
RECONNECT_TIMEOUT = 120     # 自动重连恢复超时（秒）
TRANSFER_RECOVER_TIMEOUT = 20  # 恢复 Ready 后等待起流恢复的超时（秒）

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
    print("DEV-SM-008 autoReconnect=True 异常断链自动恢复", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配（未匹配到时自动重试，最多 3 次，间隔 10s）
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS})，未匹配时最多重试 3 次（间隔 10s）...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if devices is None:
        print("[扫描] 扫描结果为空（scan 返回 None）", flush=True)
    else:
        print(f"[扫描] 发现 {len(devices)} 台设备：", flush=True)
        for d in devices:
            print(f"    - Name={getattr(d, 'Name', '?')!r}  Address={getattr(d, 'Address', '?')!r}", flush=True)

    if target is None:
        print(f"[FAIL] 未匹配到 config 中启用的设备（identity={config.TARGET_IDENTITY}）", flush=True)
        record(results, "scan 匹配到目标设备", False, f"scan 返回含 identity {config.TARGET_IDENTITY}", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OYWW1100", f"匹配到 {name} {addr}")

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

    # autoReconnect 默认值
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
        print(f"  [SensorProfile.onAutoReconnect] {s.BLEDevice.Name} restore={restore}", flush=True)
        return False  # 回退默认恢复流程（connect -> init -> replay setParam -> startDataNotification）

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
        except Exception as e:
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
        except Exception as e:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # ---- 阶段 1：人工断电/移出范围，观察断链 ----
    input("\n>>> [人工操作] 请【关闭】待测设备 OYWW1100 电源（或移出蓝牙范围），断电后保持断电，别提前开机，完成后按回车继续 ...")

    print(f"\n[等待断链] 最多 {DISCONNECT_TIMEOUT}s ...", flush=True)
    # 检查点 1：状态变化到 Disconnected（观察断链事件）
    state_disconnected = _wait_until(
        lambda: sensor.deviceState == DeviceStateEx.Disconnected,
        DISCONNECT_TIMEOUT, 0.5, "deviceState==Disconnected")
    # 检查点 2：onErrorCallback 触发（异常断链原因上报，证明是"异常断链"而非正常断开）
    error_reported = len(errors) > 0
    print(f"[断链] deviceState={sensor.deviceState} 错误回调 {len(errors)} 次 errors={errors!r} 状态序列 {[str(s) for s in states]}", flush=True)

    record(results, "断链状态变化（deviceState==Disconnected）", state_disconnected,
           "deviceState==Disconnected",
           f"state={sensor.deviceState} 状态序列={[str(s) for s in states]}")
    record(results, "onErrorCallback 触发（异常断链原因上报）", error_reported,
           "onErrorCallback 至少触发 1 次并上报断链原因",
           f"errors={errors!r}（触发 {len(errors)} 次）")

    # ---- 阶段 2：人工恢复供电，观察自动重连 ----
    input("\n>>> [人工操作] 请【重新开启】待测设备 OYWW1100 电源（或移回范围），完成后按回车继续 ...")

    print(f"\n[等待自动重连] 最多 {RECONNECT_TIMEOUT}s ...", flush=True)
    reconnected = _wait_until(
        lambda: sensor.deviceState == DeviceStateEx.Ready,
        RECONNECT_TIMEOUT, 0.5, "自动重连恢复 Ready")
    print(f"[恢复] deviceState={sensor.deviceState} onAutoReconnect {len(reconnects)} 次 状态序列 {[str(s) for s in states]}", flush=True)

    record(results, "onAutoReconnect 触发", len(reconnects) > 0,
           "onAutoReconnect 触发", f"触发 {len(reconnects)} 次 restore={reconnects!r}")
    record(results, "恢复后重新 Ready", reconnected,
           "deviceState==Ready", f"state={sensor.deviceState}")

    # 恢复起流：Ready 后 SDK 仍需时间重放 init/setParam/startDataNotification，
    # 故额外轮询等待 isDataTransfering 恢复为 True，而非立即读取
    print(f"\n[等待恢复起流] 最多 {TRANSFER_RECOVER_TIMEOUT}s ...", flush=True)
    transfer_recovered = _wait_until(
        lambda: sensor.isDataTransfering is True,
        TRANSFER_RECOVER_TIMEOUT, 0.5, "isDataTransfering 恢复为 True")
    transferring_after = sensor.isDataTransfering
    print(f"[恢复起流] isDataTransfering = {transferring_after}", flush=True)
    if reconnected:
        record(results, "恢复后 isDataTransfering==True（恢复起流）", transfer_recovered,
               "isDataTransfering == True", f"isDataTransfering == {transferring_after}")
    else:
        record(results, "恢复后 isDataTransfering==True（恢复起流）", False,
               "isDataTransfering == True", "未恢复 Ready，跳过")

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception as e:
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
