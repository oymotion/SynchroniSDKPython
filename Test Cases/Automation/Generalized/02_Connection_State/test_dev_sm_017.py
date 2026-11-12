# -*- coding: utf-8 -*-
"""DEV-SM-017：onAutoReconnectingStateChange 回调触发（sensor, is_auto_reconnecting）。

对应用例：02_连接与状态机.md -> DEV-SM-017
可自动化：semi-auto（需人工断电/移出范围，再恢复供电/移回范围）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) scan -> requireSensor -> autoReconnect=True
  3) connect 到 Ready -> init -> startDataNotification 起流
  4) 注册 onAutoReconnectingStateChange（签名 (sensor, is_auto_reconnecting)）
  5) 人工断电/移出范围 -> 观察断链
  6) 人工恢复供电/移回范围 -> 断言回调 (sensor, True) 与 (sensor, False) 各至少一次

说明：
  SDK 1.4.4 新增 onAutoReconnectingStateChange / on_auto_reconnecting_state_change，
  在自动重连开始/结束时通知上层，与 isAutoReconnecting 查询互补。
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

DISCONNECT_TIMEOUT = 60   # 断链检测超时（秒）
RECONNECT_TIMEOUT = 120   # 自动重连恢复超时（秒）


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
    print("DEV-SM-017 onAutoReconnectingStateChange 回调触发（sensor, is_auto_reconnecting）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # scan + requireSensor
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
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

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    sensor.autoReconnect = True
    print(f"[检查] SensorProfile.autoReconnect = {sensor.autoReconnect}", flush=True)

    # 注册回调（在连接后、起流前注册，覆盖完整重连过程）
    events = []

    def on_ar_state(s, is_auto_reconnecting):
        events.append((s, is_auto_reconnecting))
        pname = getattr(getattr(s, 'BLEDevice', None), 'Name', '?')
        print(f"  [SensorProfile.onAutoReconnectingStateChange] sensor={pname} is_auto_reconnecting={is_auto_reconnecting}", flush=True)

    sensor.onAutoReconnectingStateChange = on_ar_state

    # connect 到 Ready
    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True, "connect() 返回 True", f"connect() -> {connect_txt}")

    _wait_until(lambda: sensor.deviceState == DeviceStateEx.Ready, 10, 0.2, "connect 后等待 Ready")
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
    print(f"\n[init] SensorProfile.init(...) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    print("[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    # 阶段 1：人工断电，观察断链
    input("\n>>> [人工操作] 请【关闭】待测设备电源（或移出蓝牙范围），完成后按回车继续 ...")
    print(f"\n[等待断链] 最多 {DISCONNECT_TIMEOUT}s ...", flush=True)
    state_disconnected = _wait_until(lambda: sensor.deviceState == DeviceStateEx.Disconnected,
                                     DISCONNECT_TIMEOUT, 0.5, "deviceState==Disconnected")
    record(results, "断链后 deviceState==Disconnected", state_disconnected,
           "deviceState==Disconnected", f"state={sensor.deviceState}")

    # 阶段 2：人工恢复，观察回调
    input("\n>>> [人工操作] 请【重新开启】待测设备电源（或移回范围），完成后按回车继续 ...")
    print(f"\n[等待自动重连] 最多 {RECONNECT_TIMEOUT}s，等待回调 ...", flush=True)
    reconnected = _wait_until(lambda: sensor.deviceState == DeviceStateEx.Ready,
                              RECONNECT_TIMEOUT, 0.5, "deviceState==Ready")
    print(f"[恢复] deviceState={sensor.deviceState}  收到 {len(events)} 次回调 events={[v for _, v in events]!r}", flush=True)

    saw_true = any(v is True for _, v in events)
    saw_false = any(v is False for _, v in events)
    all_profile = all(isinstance(s, SensorProfile) for s, _ in events) if events else False

    record(results, "回调收到 is_auto_reconnecting==True", saw_true,
           "重连开始时回调 (sensor, True)", f"events={[v for _, v in events]!r}")
    record(results, "回调收到 is_auto_reconnecting==False", saw_false,
           "重连结束时回调 (sensor, False)", f"events={[v for _, v in events]!r}")
    record(results, "回调 sensor 参数为 SensorProfile 实例", all_profile,
           "sensor 参数为 SensorProfile 实例", f"events 数量={len(events)}")

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
