# -*- coding: utf-8 -*-
"""MISC-FUNC-010：submit 执行阻塞式连接/断开——不阻塞调用方，结果与同步一致。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-010
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor
  3) fut = submit(sensor.connect) —— 断言 submit 调用立即返回（不阻塞）
  4) fut.result(timeout) == True，deviceState == Ready
  5) fut = submit(sensor.disconnect) —— fut.result(timeout) == True，deviceState == Disconnected
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from sensor import submit  # 被测公开 API：后台线程执行器
import config
from common import record, scan_and_match


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-010 submit 执行阻塞式连接/断开", flush=True)
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
    print(f"\n[扫描] ctrl.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
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
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # ---- submit(sensor.connect)：非阻塞提交 ----
    print("\n[连接] fut = submit(sensor.connect) ...", flush=True)
    t0 = time.time()
    fut = submit(sensor.connect)
    submit_cost = time.time() - t0
    non_blocking = (submit_cost < 0.5)
    print(f"[检查1] submit(connect) 调用耗时 {submit_cost:.4f}s（应远小于 connect 实际耗时）", flush=True)
    record(results, "submit(connect) 立即返回（不阻塞）", non_blocking,
           "submit 调用耗时 < 0.5s", f"submit 耗时 {submit_cost:.4f}s")

    try:
        ok = fut.result(timeout=40)
    except Exception as e:
        ok = None
        print(f"[连接] fut.result 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[检查2] submit(connect).result(timeout) = {ok}", flush=True)
    record(results, "submit(connect) 结果 True", ok is True,
           "result() == True", f"result() == {ok}")

    # 等待 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    state_ready = sensor.deviceState
    ready_ok = (state_ready == DeviceStateEx.Ready)
    print(f"[检查3] connect 后 deviceState = {state_ready}", flush=True)
    record(results, "connect 后 deviceState==Ready", ready_ok,
           "deviceState == DeviceStateEx.Ready", f"deviceState == {state_ready}")

    # ---- submit(sensor.disconnect) ----
    print("\n[断开] fut = submit(sensor.disconnect) ...", flush=True)
    fut = submit(sensor.disconnect)
    try:
        ok = fut.result(timeout=25)
    except Exception as e:
        ok = None
        print(f"[断开] fut.result 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[检查4] submit(disconnect).result(timeout) = {ok}", flush=True)
    record(results, "submit(disconnect) 结果 True", ok is True,
           "result() == True", f"result() == {ok}")

    # 等待 Disconnected
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Disconnected:
        time.sleep(0.2)
    state_disc = sensor.deviceState
    disc_ok = (state_disc == DeviceStateEx.Disconnected)
    print(f"[检查5] disconnect 后 deviceState = {state_disc}", flush=True)
    record(results, "disconnect 后 deviceState==Disconnected", disc_ok,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {state_disc}")

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
