# -*- coding: utf-8 -*-
"""MISC-FUNC-006：SensorData.startSampleIndex 属性存在且值合法。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-006
可自动化：auto（需待测设备上电在范围内，起流获取多批 SensorData）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OYWW1100 -> requireSensor -> connect -> init -> startDataNotification
  3) 连续收到多批 SensorData，检查 startSampleIndex 属性存在、为非负整数（≥0）

注意：SDK 未文档化 startSampleIndex 的单调性保证，此处不做单调性断言。
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


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-006 SensorData.startSampleIndex 属性存在且值合法", flush=True)
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

    print("\n[异步连接] await sensor.asyncConnect() ...", flush=True)
    try:
        ok = await sensor.asyncConnect()
    except Exception as e:
        ok = False
        print(f"[异步连接] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok is not True:
        print("[FAIL] asyncConnect 失败", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        await asyncio.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print("[FAIL] 未到达 Ready", flush=True)
        ctrl.terminate()
        return

    print(f"\n[异步init] await sensor.asyncInit({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        ok_init = await sensor.asyncInit(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        ok_init = False
        print(f"[异步init] 抛异常 {type(e).__name__}: {e}", flush=True)
    if ok_init is not True:
        print("[FAIL] asyncInit 失败", flush=True)
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 收集多批 SensorData
    batch_lock = threading.Lock()
    batches = []  # [(startSampleIndex, first_sample_index), ...]

    def on_data(s, data_list):
        for d in data_list:
            cs = getattr(d, 'channelSamples', None)
            if cs and len(cs) > 0 and len(cs[0]) > 0:
                ssi = getattr(d, 'startSampleIndex', None)
                first_si = None
                try:
                    first_si = cs[0][0].sampleIndex
                except Exception:
                    pass
                with batch_lock:
                    batches.append((ssi, first_si))

    sensor.onDataCallback = on_data

    print(f"\n[异步起流] await sensor.asyncStartDataNotification() ...", flush=True)
    try:
        start_ok = await sensor.asyncStartDataNotification()
    except Exception as e:
        start_ok = False
        print(f"[异步起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    if start_ok is not True:
        print("[FAIL] asyncStartDataNotification 失败", flush=True)
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    print(f"\n[等待] 采集数据 {COLLECT_SECONDS}s ...", flush=True)
    await asyncio.sleep(COLLECT_SECONDS)

    try:
        await sensor.asyncStopDataNotification()
    except Exception as e:
        print(f"[异步停流] 抛异常 {type(e).__name__}: {e}", flush=True)

    n_batches = len(batches)
    print(f"\n[收集] 共收到 {n_batches} 批数据", flush=True)

    if n_batches == 0:
        print("[FAIL] 未收到任何 SensorData", flush=True)
        record(results, "收到 ≥1 批 SensorData", False, "≥1 批", "0 批")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "收到 ≥1 批 SensorData", True, "≥1 批", f"{n_batches} 批")

    # 打印前几批的 startSampleIndex
    print(f"\n[批次详情] 前 5 批:", flush=True)
    for i, (ssi, first_si) in enumerate(batches[:5]):
        print(f"  批[{i}]: startSampleIndex={ssi!r} first_sample.sampleIndex={first_si!r}", flush=True)

    # 检查：startSampleIndex 属性存在且为非负整数
    all_valid = True
    for i, (ssi, first_si) in enumerate(batches):
        if ssi is None:
            print(f"[属性检查] 批[{i}]: startSampleIndex = None", flush=True)
            all_valid = False
        elif not isinstance(ssi, (int, float)):
            print(f"[属性检查] 批[{i}]: startSampleIndex 类型异常 = {type(ssi).__name__}({ssi!r})", flush=True)
            all_valid = False
        elif ssi < 0:
            print(f"[属性检查] 批[{i}]: startSampleIndex = {ssi} < 0", flush=True)
            all_valid = False
    record(results, "startSampleIndex 属性存在且为非负整数", all_valid,
           "所有批次的 startSampleIndex 为非负整数", "全部合法" if all_valid else "存在非法值")

    # 清理
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