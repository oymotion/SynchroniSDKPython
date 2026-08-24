# -*- coding: utf-8 -*-
"""DEV-SM-013：isReady 与 deviceState==Ready 一致。

对应用例：02_连接与状态机.md -> DEV-SM-013
可自动化：auto（无需人工介入，但需设备开机在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan -> requireSensor，读取 connect 前的 deviceState 与 isReady
  3) connect 到 Ready，读取 Ready 时的 deviceState 与 isReady
  4) disconnect，读取断开后的 deviceState 与 isReady
  5) 在三个状态点断言一致性：isReady == (deviceState == Ready)

可证伪点：isReady 与 deviceState 不同步（如 isReady 滞后、或两者对 Ready 的判定不一致）。
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match

READY_TIMEOUT = 15     # 连接后等待 Ready 超时（秒）
DISCONNECT_TIMEOUT = 15  # 断开后等待非 Ready 超时（秒）


def _wait_until(cond, timeout, interval=0.5, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(interval)
    print(f"  [等待超时] {what}（{timeout}s）", flush=True)
    return False


def _consistent(is_ready, state):
    """isReady 与 deviceState==Ready 的一致性。"""
    return (is_ready is True) == (state == DeviceStateEx.Ready)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-013 isReady 与 deviceState==Ready 一致", flush=True)
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

    # ---- 状态点 1：connect 前（初始） ----
    s0 = sensor.deviceState
    r0 = sensor.isReady
    print(f"\n[初始] deviceState={s0} isReady={r0}", flush=True)
    record(results, "connect 前 deviceState==Disconnected", s0 == DeviceStateEx.Disconnected,
           "deviceState == Disconnected", f"deviceState == {s0}")
    record(results, "connect 前 isReady==False", r0 is False,
           "isReady == False", f"isReady == {r0}")
    record(results, "connect 前一致性 isReady==(deviceState==Ready)", _consistent(r0, s0),
           "isReady == (deviceState == Ready)", f"isReady={r0} deviceState={s0}")

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

    _wait_until(lambda: sensor.deviceState == DeviceStateEx.Ready, READY_TIMEOUT, 0.2, "连接后等待 Ready")
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

    # ---- 状态点 2：Ready ----
    s1 = sensor.deviceState
    r1 = sensor.isReady
    print(f"\n[Ready] deviceState={s1} isReady={r1}", flush=True)
    record(results, "Ready 时 isReady==True", r1 is True,
           "isReady == True", f"isReady == {r1}")
    record(results, "Ready 时一致性 isReady==(deviceState==Ready)", _consistent(r1, s1),
           "isReady == (deviceState == Ready)", f"isReady={r1} deviceState={s1}")

    # disconnect
    print("\n[断开] SensorProfile.disconnect() ...", flush=True)
    try:
        dret = sensor.disconnect()
        disc_txt = f"返回 {dret}"
    except Exception as e:
        dret = None
        disc_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[断开] SensorProfile.disconnect() -> {disc_txt}", flush=True)

    _wait_until(lambda: sensor.deviceState != DeviceStateEx.Ready, DISCONNECT_TIMEOUT, 0.2, "断开后等待非 Ready")

    # ---- 状态点 3：断开后 ----
    s2 = sensor.deviceState
    r2 = sensor.isReady
    print(f"[断开后] deviceState={s2} isReady={r2}", flush=True)
    record(results, "断开后 deviceState != Ready", s2 != DeviceStateEx.Ready,
           "deviceState != Ready", f"deviceState == {s2}")
    record(results, "断开后 isReady==False", r2 is False,
           "isReady == False", f"isReady == {r2}")
    record(results, "断开后一致性 isReady==(deviceState==Ready)", _consistent(r2, s2),
           "isReady == (deviceState == Ready)", f"isReady={r2} deviceState={s2}")

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
