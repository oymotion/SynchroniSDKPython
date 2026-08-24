# -*- coding: utf-8 -*-
"""ASYNC-FUNC-008：asyncStartDataNotification 起流成功，回调收到数据，asyncStopDataNotification 停流成功。

对应用例：09_异步接口.md -> ASYNC-FUNC-008
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor
  3) await sensor.asyncConnect() -> 到达 Ready
  4) await sensor.asyncInit(20, 1000)
  5) 注册 onDataCallback 收集数据
  6) await sensor.asyncStartDataNotification() -> 断言返回 True，isDataTransfering==True
  7) 等待 3 秒，检查回调是否收到数据
  8) await sensor.asyncStopDataNotification() -> 断言返回 True
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

COLLECT_SECONDS = 3


class DataResult:
    def __init__(self):
        self.batches = 0
        self.total_samples = 0


def make_on_data(result):
    def on_data(sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            result.batches += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            result.total_samples += n
    return on_data


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("ASYNC-FUNC-008 asyncStartDataNotification 起流/停流成功，回调收到数据", flush=True)
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
    state = sensor.deviceState
    ready = (state == DeviceStateEx.Ready)
    print(f"[检查1] 连接后 deviceState = {state}", flush=True)
    record(results, "asyncConnect 后 deviceState==Ready", ready,
           "deviceState == DeviceStateEx.Ready", f"deviceState == {state}")

    if not ready:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # asyncInit
    print("\n[异步初始化] await sensor.asyncInit(20, 1000) ...", flush=True)
    try:
        ok_init = await sensor.asyncInit(20, 1000)
    except Exception as e:
        ok_init = False
        print(f"[异步初始化] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步初始化] sensor.asyncInit(20, 1000) -> {ok_init}", flush=True)
    record(results, "asyncInit 返回 True", ok_init is True,
           "asyncInit(20, 1000) 返回 True", f"asyncInit() -> {ok_init}")

    # 注册数据回调
    data_result = DataResult()
    sensor.onDataCallback = make_on_data(data_result)

    # ---- asyncStartDataNotification ----
    print("\n[异步起流] await sensor.asyncStartDataNotification() ...", flush=True)
    try:
        start_ok = await sensor.asyncStartDataNotification()
    except Exception as e:
        start_ok = False
        print(f"[异步起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步起流] sensor.asyncStartDataNotification() -> {start_ok}", flush=True)
    record(results, "asyncStartDataNotification 返回 True", start_ok is True,
           "asyncStartDataNotification() 返回 True", f"asyncStartDataNotification() -> {start_ok}")

    transferring = sensor.isDataTransfering
    print(f"[检查2] 起流后 isDataTransfering = {transferring}", flush=True)
    record(results, "起流后 isDataTransfering==True", transferring is True,
           "isDataTransfering == True", f"isDataTransfering == {transferring}")

    # 采集数据
    print(f"\n[采集] 等待 {COLLECT_SECONDS}s 观察 onDataCallback ...", flush=True)
    await asyncio.sleep(COLLECT_SECONDS)
    print(f"[采集] 批数={data_result.batches} 样本数={data_result.total_samples}", flush=True)
    record(results, "onDataCallback 收到 >=1 批数据", data_result.batches >= 1,
           "onDataCallback 收到 >=1 批数据", f"批数={data_result.batches} 样本数={data_result.total_samples}")

    # ---- asyncStopDataNotification ----
    print("\n[异步停流] await sensor.asyncStopDataNotification() ...", flush=True)
    try:
        stop_ok = await sensor.asyncStopDataNotification()
    except Exception as e:
        stop_ok = False
        print(f"[异步停流] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步停流] sensor.asyncStopDataNotification() -> {stop_ok}", flush=True)
    record(results, "asyncStopDataNotification 返回 True", stop_ok is True,
           "asyncStopDataNotification() 返回 True", f"asyncStopDataNotification() -> {stop_ok}")

    # 清理
    sensor.onDataCallback = None
    try:
        await sensor.asyncDisconnect()
    except Exception as e:
        print(f"[断开] 抛异常 {type(e).__name__}: {e}", flush=True)

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