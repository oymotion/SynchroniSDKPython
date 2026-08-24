# -*- coding: utf-8 -*-
"""DEV-SM-002：connect 失败返回 False 并回调原因。

对应用例：02_连接与状态机.md -> DEV-SM-002
可自动化：semi-auto（需人工关闭设备电源/移出范围，制造连接失败）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 需先开机（用于扫描获取 BLEDevice），再关闭电源制造连接失败

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OYWW1100 -> 记录 BLEDevice
  3) 人工关闭设备电源（或移出范围）-> 按回车
  4) requireSensor(已记录的 BLEDevice)
  5) 注册 onErrorCallback / onStateChanged
  6) connect() -> 断言返回 False
  7) 断言 onErrorCallback 收到原因、未进入 Ready
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
    print("DEV-SM-002 connect 失败返回 False 并回调原因", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 需先开机（扫描获取 BLEDevice），再关闭电源制造连接失败", flush=True)

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # ---- 阶段 1：设备开机时扫描获取 BLEDevice ----
    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，完成后按回车继续 ...")

    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备（OYWW1100/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OYWW1100", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OYWW1100", f"匹配到 {name} {addr}")

    # ---- 阶段 2：人工关闭设备，制造连接失败 ----
    input("\n>>> [人工操作] 请【关闭】待测设备 OYWW1100 电源（或将其移出蓝牙范围），完成后按回车继续 ...")

    # requireSensor（用阶段1扫描到的 BLEDevice）
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # 注册回调
    errors = []
    states = []

    def on_error(s, reason):
        errors.append(reason)
        print(f"  [SensorProfile.onErrorCallback] {s.BLEDevice.Name} reason={reason!r}", flush=True)

    def on_state(s, st):
        states.append(st)
        print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)

    sensor.onErrorCallback = on_error
    sensor.onStateChanged = on_state

    # connect（对已关机设备，可能阻塞至 BLE 连接超时）
    print("\n[连接] SensorProfile.connect() ...（对已关机设备，可能阻塞至连接超时，请耐心等待）", flush=True)
    t0 = time.time()
    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    elapsed = round(time.time() - t0, 1)
    print(f"[连接] SensorProfile.connect() -> {connect_txt}（耗时 {elapsed}s）", flush=True)

    # 判定1：connect 返回 False
    record(results, "SensorProfile.connect 返回 False", ok is False,
           "connect() 返回 False", f"connect() -> {connect_txt}")

    # 判定2：onErrorCallback 收到原因
    got_error = len(errors) > 0
    print(f"[检查] SensorProfile.onErrorCallback 触发次数 = {len(errors)}，原因 = {errors!r}", flush=True)
    record(results, "onErrorCallback 收到原因", got_error,
           "onErrorCallback 触发且 reason 非空", f"触发 {len(errors)} 次: {errors!r}")

    # 判定3：未进入 Ready
    final_state = sensor.deviceState
    print(f"[检查] 最终 SensorProfile.deviceState = {final_state}", flush=True)
    record(results, "未进入 Ready", final_state != DeviceStateEx.Ready,
           "deviceState != DeviceStateEx.Ready", f"deviceState == {final_state}")

    # 清理（若意外连接成功则断开）
    if final_state == DeviceStateEx.Ready:
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
