# -*- coding: utf-8 -*-
"""MISC-FUNC-003：SensorData.clear/reset 复用。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-003
可自动化：auto（需待测设备上电在范围内，起流获取 SensorData 对象）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor -> connect -> init -> startDataNotification
  3) 从 onDataCallback 获取一个 SensorData 对象
  4) 调用 clear()（或 reset()）后检查 channelSamples/样本计数归零
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
    print("MISC-FUNC-003 SensorData.clear/reset 复用", flush=True)
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

    # 收集第一个 SensorData
    sample_lock = threading.Lock()
    captured_data = None

    def on_data(s, data_list):
        nonlocal captured_data
        for d in data_list:
            cs = getattr(d, 'channelSamples', None)
            if cs and len(cs) > 0 and len(cs[0]) > 0:
                with sample_lock:
                    if captured_data is None:
                        captured_data = d
                        print(f"[数据] 捕获到一个 SensorData, type={type(d).__name__}", flush=True)

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

    print(f"\n[等待] 等待数据到达，最多 {COLLECT_SECONDS}s ...", flush=True)
    await asyncio.sleep(COLLECT_SECONDS)

    try:
        await sensor.asyncStopDataNotification()
    except Exception as e:
        print(f"[异步停流] 抛异常 {type(e).__name__}: {e}", flush=True)

    if captured_data is None:
        print("[FAIL] 未捕获到任何 SensorData", flush=True)
        record(results, "捕获到 SensorData 对象", False, "onDataCallback 收到非空 SensorData", "未捕获到")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "捕获到 SensorData 对象", True, "onDataCallback 收到非空 SensorData",
           f"type={type(captured_data).__name__}")

    # 记录 clear 前的状态
    cs_before = getattr(captured_data, 'channelSamples', None)
    try:
        n_ch_before = len(cs_before) if cs_before else 0
    except TypeError:
        n_ch_before = 0
    print(f"[clear前] channelSamples 通道数={n_ch_before}", flush=True)

    # 尝试调用 clear() 或 reset()
    clear_method = None
    if hasattr(captured_data, 'clear'):
        clear_method = 'clear'
    elif hasattr(captured_data, 'reset'):
        clear_method = 'reset'
    else:
        print("[FAIL] SensorData 没有 clear() 或 reset() 方法", flush=True)
        record(results, "SensorData 支持 clear() 或 reset()", False, "存在 clear() 或 reset()", "两者都不存在")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "SensorData 支持 clear() 或 reset()", True, "存在 clear() 或 reset()", f"方法: {clear_method}()")

    print(f"\n[清空] 调用 SensorData.{clear_method}() ...", flush=True)
    try:
        getattr(captured_data, clear_method)()
        print(f"[清空] SensorData.{clear_method}() 执行完成", flush=True)
    except Exception as e:
        print(f"[清空] 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, f"SensorData.{clear_method}() 不抛异常", False,
               f"{clear_method}() 正常执行", f"抛异常 {type(e).__name__}: {e}")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, f"SensorData.{clear_method}() 不抛异常", True, f"{clear_method}() 正常执行", "无异常")

    # 检查 clear 后的状态
    cs_after = getattr(captured_data, 'channelSamples', None)
    if cs_after is None:
        n_ch_after = 0
        print(f"[clear后] channelSamples = None", flush=True)
    else:
        try:
            n_ch_after = len(cs_after)
        except TypeError:
            n_ch_after = 0
        print(f"[clear后] channelSamples 通道数={n_ch_after}", flush=True)

    cleared = (n_ch_after == 0 or cs_after is None)
    record(results, f"{clear_method}() 后 channelSamples 归零/空", cleared,
           "channelSamples 为空或通道数为 0", f"通道数={n_ch_after}")

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