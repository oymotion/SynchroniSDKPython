# -*- coding: utf-8 -*-
"""ASYNC-FUNC-003：asyncDisconnect 成功。

对应用例：09_异步接口.md -> ASYNC-FUNC-003
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor
  3) await sensor.asyncConnect() -> 到达 Ready
  4) await sensor.asyncDisconnect() -> 断言返回 True
  5) 断言 deviceState==Disconnected
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
    print("ASYNC-FUNC-003 asyncDisconnect 成功", flush=True)
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

    # asyncConnect
    print("\n[异步连接] await sensor.asyncConnect() ...", flush=True)
    try:
        ok = await sensor.asyncConnect()
    except Exception as e:
        ok = False
        print(f"[异步连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] asyncConnect 失败，无法继续测试 asyncDisconnect", flush=True)
        record(results, "asyncConnect 返回 True", ok is True,
               "asyncConnect() 返回 True", f"asyncConnect() -> {ok}")
        ctrl.terminate()
        return
    record(results, "asyncConnect 返回 True", ok is True,
           "asyncConnect() 返回 True", f"asyncConnect() -> {ok}")

    state = sensor.deviceState
    print(f"[检查1] 连接后 deviceState = {state}", flush=True)
    record(results, "asyncConnect 后 deviceState==Ready", state == DeviceStateEx.Ready,
           "deviceState == DeviceStateEx.Ready", f"deviceState == {state}")

    # asyncDisconnect
    print("\n[异步断开] await sensor.asyncDisconnect() ...", flush=True)
    try:
        ok_disconnect = await sensor.asyncDisconnect()
    except Exception as e:
        ok_disconnect = False
        print(f"[异步断开] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步断开] sensor.asyncDisconnect() -> {ok_disconnect}", flush=True)
    record(results, "asyncDisconnect 返回 True", ok_disconnect is True,
           "asyncDisconnect() 返回 True", f"asyncDisconnect() -> {ok_disconnect}")

    final_state = sensor.deviceState
    print(f"[检查2] 断开后 deviceState = {final_state}", flush=True)
    record(results, "asyncDisconnect 后 deviceState==Disconnected", final_state == DeviceStateEx.Disconnected,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {final_state}")

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