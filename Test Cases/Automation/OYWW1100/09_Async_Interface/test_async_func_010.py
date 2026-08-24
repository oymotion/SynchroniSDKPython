# -*- coding: utf-8 -*-
"""ASYNC-FUNC-010：async/sync 混合调用——异步连接、同步 init/起流、异步停流、同步断开，状态切换正确。

对应用例：09_异步接口.md -> ASYNC-FUNC-010
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OYWW1100 -> requireSensor
  3) await sensor.asyncConnect()（异步连接）-> 到达 Ready
  4) sensor.init(20, 1000)（同步 init）
  5) sensor.startDataNotification()（同步起流）
  6) 等待 2 秒，检查 isDataTransfering==True
  7) await sensor.asyncStopDataNotification()（异步停流）
  8) 检查 isDataTransfering==False
  9) sensor.disconnect()（同步断开）
  10) 检查 deviceState 各阶段转换正确：Connected -> Ready -> Streaming -> Disconnected
"""

import asyncio
import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match, async_scan_and_match

COLLECT_SECONDS = 2


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("ASYNC-FUNC-010 async/sync 混合调用——状态切换正确", flush=True)
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

    # ---- asyncConnect（异步连接） ----
    print("\n[异步连接] await sensor.asyncConnect() ...", flush=True)
    try:
        ok = await sensor.asyncConnect()
    except Exception as e:
        ok = False
        print(f"[异步连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] asyncConnect 失败，无法继续测试", flush=True)
        record(results, "asyncConnect 返回 True", ok is True,
               "asyncConnect() 返回 True", f"asyncConnect() -> {ok}")
        ctrl.terminate()
        return
    record(results, "asyncConnect 返回 True", ok is True,
           "asyncConnect() 返回 True", f"asyncConnect() -> {ok}")

    # 等待 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    state_connected = sensor.deviceState
    ready_ok = (state_connected == DeviceStateEx.Ready)
    print(f"[检查1] asyncConnect 后 deviceState = {state_connected}", flush=True)
    record(results, "asyncConnect 后 deviceState==Ready", ready_ok,
           "deviceState == DeviceStateEx.Ready", f"deviceState == {state_connected}")

    if not ready_ok:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # ---- sensor.init（同步 init） ----
    print("\n[同步初始化] sensor.init(20, 1000) ...", flush=True)
    try:
        ok_init = sensor.init(20, 1000)
    except Exception as e:
        ok_init = False
        print(f"[同步初始化] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[同步初始化] sensor.init(20, 1000) -> {ok_init}", flush=True)
    record(results, "sync init 返回 True", ok_init is True,
           "init(20, 1000) 返回 True", f"init() -> {ok_init}")

    # ---- sensor.startDataNotification（同步起流） ----
    print("\n[同步起流] sensor.startDataNotification() ...", flush=True)
    try:
        start_ok = sensor.startDataNotification()
    except Exception as e:
        start_ok = False
        print(f"[同步起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[同步起流] sensor.startDataNotification() -> {start_ok}", flush=True)
    record(results, "sync startDataNotification 返回 True", start_ok is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_ok}")

    # 等待 2 秒，检查 isDataTransfering
    print(f"\n[采集] 等待 {COLLECT_SECONDS}s ...", flush=True)
    await asyncio.sleep(COLLECT_SECONDS)
    transferring_after_start = sensor.isDataTransfering
    print(f"[检查2] 起流后 isDataTransfering = {transferring_after_start}", flush=True)
    record(results, "起流后 isDataTransfering==True", transferring_after_start is True,
           "isDataTransfering == True", f"isDataTransfering == {transferring_after_start}")

    # ---- await sensor.asyncStopDataNotification（异步停流） ----
    print("\n[异步停流] await sensor.asyncStopDataNotification() ...", flush=True)
    try:
        stop_ok = await sensor.asyncStopDataNotification()
    except Exception as e:
        stop_ok = False
        print(f"[异步停流] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步停流] sensor.asyncStopDataNotification() -> {stop_ok}", flush=True)
    record(results, "asyncStopDataNotification 返回 True", stop_ok is True,
           "asyncStopDataNotification() 返回 True", f"asyncStopDataNotification() -> {stop_ok}")

    transferring_after_stop = sensor.isDataTransfering
    print(f"[检查3] 停流后 isDataTransfering = {transferring_after_stop}", flush=True)
    record(results, "停流后 isDataTransfering==False", transferring_after_stop is False,
           "isDataTransfering == False", f"isDataTransfering == {transferring_after_stop}")

    # ---- sensor.disconnect（同步断开） ----
    print("\n[同步断开] sensor.disconnect() ...", flush=True)
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[同步断开] 抛异常 {type(e).__name__}: {e}", flush=True)

    # 等待状态传播
    time.sleep(1)
    state_disconnected = sensor.deviceState
    disconnected_ok = (state_disconnected == DeviceStateEx.Disconnected)
    print(f"[检查4] disconnect 后 deviceState = {state_disconnected}", flush=True)
    record(results, "sync disconnect 后 deviceState==Disconnected", disconnected_ok,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {state_disconnected}")

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