# -*- coding: utf-8 -*-
"""ASYNC-FUNC-002：asyncConnect 成功/失败。

对应用例：09_异步接口.md -> ASYNC-FUNC-002
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OYWW1100 -> requireSensor
  3) await sensor.asyncConnect() -> 断言返回 True，deviceState==Ready
  4) disconnect
  5) 对无效设备（None）调用 asyncConnect -> 断言返回 False
"""

import asyncio
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match, async_scan_and_match


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("ASYNC-FUNC-002 asyncConnect 成功/失败", flush=True)
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

    # 扫描匹配
    print(f"\n[扫描] await ctrl.asyncScan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = await async_scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
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

    # ---- asyncConnect 成功场景 ----
    print("\n[异步连接] await sensor.asyncConnect() ...", flush=True)
    try:
        ok = await sensor.asyncConnect()
    except Exception as e:
        ok = False
        print(f"[异步连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步连接] sensor.asyncConnect() -> {ok}", flush=True)
    record(results, "asyncConnect 返回 True", ok is True,
           "asyncConnect() 返回 True", f"asyncConnect() -> {ok}")

    state = sensor.deviceState
    print(f"[检查1] 连接后 deviceState = {state}", flush=True)
    record(results, "asyncConnect 后 deviceState==Ready", state == DeviceStateEx.Ready,
           "deviceState == DeviceStateEx.Ready", f"deviceState == {state}")

    # 断开以便后续测试
    print("\n[断开] await sensor.asyncDisconnect() ...", flush=True)
    try:
        await sensor.asyncDisconnect()
    except Exception as e:
        print(f"[断开] 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- asyncConnect 失败场景（无效设备） ----
    # 构造一个不存在的 MAC 地址，创建无效 sensor
    print("\n[异步连接-失败] 构造无效设备，调用 asyncConnect ...", flush=True)
    invalid_device = BLEDevice("FAKE", "00:00:00:00:00:00", 0)
    invalid_sensor = ctrl.requireSensor(invalid_device)
    if invalid_sensor is None:
        print("[异步连接-失败] requireSensor(无效设备) 返回 None，无法构造无效设备", flush=True)
        record(results, "对无效设备 asyncConnect 返回 False", None,
               "asyncConnect() 返回 False", "requireSensor 返回 None，无法构造无效设备")
    else:
        try:
            ok_fail = await invalid_sensor.asyncConnect()
        except Exception as e:
            ok_fail = False
            print(f"[异步连接-失败] 抛异常 {type(e).__name__}: {e}", flush=True)
        print(f"[异步连接-失败] asyncConnect() -> {ok_fail}", flush=True)
        record(results, "对无效设备 asyncConnect 返回 False", ok_fail is False,
               "asyncConnect() 返回 False", f"asyncConnect() -> {ok_fail}")

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
    asyncio.run(main_async())