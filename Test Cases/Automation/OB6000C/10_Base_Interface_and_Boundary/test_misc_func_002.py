# -*- coding: utf-8 -*-
"""MISC-FUNC-002：Sample.reset 清空字段。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-002
可自动化：auto（需待测设备上电在范围内，起流获取 Sample 对象）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor -> connect -> init -> startDataNotification
  3) 从 onDataCallback 获取一个 Sample 对象
  4) 调用 sample.reset() 后检查各字段归零/复位
"""

import os
import sys
import time
import threading
import asyncio

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match, async_scan_and_match

COLLECT_SECONDS = 5

SAMPLE_FIELDS = ["data", "rawData", "impedance", "saturation",
                 "sampleIndex", "channelIndex", "timeStampInMs", "absTimeStampInSec", "isLost"]


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-002 Sample.reset 清空字段", flush=True)
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
    target_ids = config.TARGET_IDENTITY.split(",") if hasattr(config, 'TARGET_IDENTITY') else ["?"]
    print(f"\n[扫描] 目标 identity: {target_ids}（config.TARGET_IDENTITY = '{config.TARGET_IDENTITY}'）", flush=True)
    target, devices = await async_scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    scanned_list = [(getattr(d, 'Name', '?'), getattr(d, 'Address', '?')) for d in (devices or [])]
    print(f"[扫描] 扫描到 {len(scanned_list)} 台设备: {scanned_list}", flush=True)

    if target is None:
        print(f"[FAIL] 未匹配到目标设备（目标 identity: {target_ids}，扫描到: {scanned_list}）", flush=True)
        print("[提示] 请检查 config.py 中 TARGET_IDENTITY 是否设置为正确的设备 identity。", flush=True)
        if hasattr(config, 'DEVICES'):
            available_ids = [getattr(d, 'identity', '?') for d in config.DEVICES]
            print(f"[提示] 当前 DEVICES 配置中可用的 identity: {available_ids}", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备",
               f"未匹配到目标（目标: {target_ids}，扫描到: {scanned_list}）")
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

    # requireSensor -> connect -> init -> startDataNotification
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        ctrl.terminate()
        return

    # 异步连接
    print("\n[异步连接] await sensor.asyncConnect() ...", flush=True)
    try:
        ok = await sensor.asyncConnect()
    except Exception as e:
        ok = False
        print(f"[异步连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] asyncConnect 失败", flush=True)
        record(results, "asyncConnect 返回 True", ok is True, "asyncConnect() 返回 True", f"asyncConnect() -> {ok}")
        ctrl.terminate()
        return

    # 等待 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        await asyncio.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print("[FAIL] 未到达 Ready", flush=True)
        ctrl.terminate()
        return

    # 异步 init
    print(f"\n[异步init] await sensor.asyncInit({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        ok_init = await sensor.asyncInit(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        ok_init = False
        print(f"[异步init] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok_init is not True:
        print("[FAIL] asyncInit 失败", flush=True)
        record(results, "asyncInit 返回 True", ok_init is True, "asyncInit() 返回 True", f"asyncInit() -> {ok_init}")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 收集第一个 Sample
    sample_lock = threading.Lock()
    captured_sample = None

    def on_data(s, data_list):
        nonlocal captured_sample
        for d in data_list:
            cs = getattr(d, 'channelSamples', None)
            if cs and len(cs) > 0 and len(cs[0]) > 0:
                with sample_lock:
                    if captured_sample is None:
                        captured_sample = cs[0][0]
                        print(f"[数据] 捕获到一个 Sample, type={type(captured_sample).__name__}", flush=True)

    sensor.onDataCallback = on_data

    print(f"\n[异步起流] await sensor.asyncStartDataNotification() ...", flush=True)
    try:
        start_ok = await sensor.asyncStartDataNotification()
    except Exception as e:
        start_ok = False
        print(f"[异步起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    if start_ok is not True:
        print("[FAIL] asyncStartDataNotification 失败", flush=True)
        record(results, "asyncStartDataNotification 返回 True", start_ok is True,
               "asyncStartDataNotification() 返回 True", f"asyncStartDataNotification() -> {start_ok}")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 等待数据
    print(f"\n[等待] 等待数据到达，最多 {COLLECT_SECONDS}s ...", flush=True)
    await asyncio.sleep(COLLECT_SECONDS)

    try:
        await sensor.asyncStopDataNotification()
    except Exception as e:
        print(f"[异步停流] 抛异常 {type(e).__name__}: {e}", flush=True)

    if captured_sample is None:
        print("[FAIL] 未捕获到任何 Sample", flush=True)
        record(results, "捕获到 Sample 对象", False, "onDataCallback 收到非空 Sample", "未捕获到 Sample")
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "捕获到 Sample 对象", True, "onDataCallback 收到非空 Sample", f"type={type(captured_sample).__name__}")

    # 记录 reset 前的字段值
    before = {}
    for f in SAMPLE_FIELDS:
        try:
            before[f] = getattr(captured_sample, f, None)
        except Exception as e:
            before[f] = f"读取异常: {e}"
    print(f"\n[reset前] 字段值: {before}", flush=True)

    # 调用 reset()
    print("\n[reset] 调用 sample.reset() ...", flush=True)
    try:
        captured_sample.reset()
        print("[reset] sample.reset() 执行完成", flush=True)
    except Exception as e:
        print(f"[reset] 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, "sample.reset() 不抛异常", False, "reset() 正常执行", f"抛异常 {type(e).__name__}: {e}")
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "sample.reset() 不抛异常", True, "reset() 正常执行", "无异常")

    # 检查 reset 后的字段值
    after = {}
    for f in SAMPLE_FIELDS:
        try:
            after[f] = getattr(captured_sample, f, None)
        except Exception as e:
            after[f] = f"读取异常: {e}"
    print(f"[reset后] 字段值: {after}", flush=True)

    # 各字段验证：reset 后应为初始/空值（0 或 None，即"复位"到未赋值状态）
    reset_fields = [
        "data", "rawData", "impedance", "saturation",
        "sampleIndex", "channelIndex", "timeStampInMs",
        "absTimeStampInSec",
    ]
    for f in reset_fields:
        v = after.get(f)
        ok = (v is None or v == 0 or v == 0.0)
        record(results, f"reset 后 sample.{f} 为初始值（0 或 None）", ok,
               f"sample.{f} == 0 或 None", f"sample.{f} = {v!r}")

    # isLost 应为 False 或 None
    v_lost = after.get("isLost")
    record(results, "reset 后 sample.isLost 为 False 或 None", v_lost is False or v_lost is None,
           "sample.isLost == False 或 None", f"sample.isLost = {v_lost!r}")

    # 清理
    try:
        await sensor.asyncDisconnect()
    except Exception as e:
        print(f"[异步断开] 抛异常 {type(e).__name__}: {e}", flush=True)

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