# -*- coding: utf-8 -*-
"""MISC-FUNC-004：SensorData.to_flatbuffers/from_flatbuffers 往返。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-004
可自动化：auto（需待测设备上电在范围内，起流获取 SensorData 对象）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor -> connect -> init -> startDataNotification
  3) 从 onDataCallback 获取一个 SensorData 对象
  4) 调 to_flatbuffers() 得 bytes，再 from_flatbuffers(bytes) 还原
  5) 比对 getDataType/getDeviceMac/getSampleCount/各样本 data/rawData/sampleIndex 一致
"""

import os
import sys
import time
import threading
import asyncio
import inspect

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match, async_scan_and_match

COLLECT_SECONDS = 5

SAMPLE_FIELDS = ["data", "rawData", "sampleIndex"]


def _sensor_data_snapshot(d):
    """获取 SensorData 的字段快照，用于序列化往返比对。"""
    snap = {}
    methods = {
        "getDataType": lambda: d.getDataType(),
        "getDeviceMac": lambda: d.getDeviceMac(),
        "getSampleCount": lambda: d.getSampleCount(),
    }
    for m, fn in methods.items():
        try:
            snap[m] = fn()
        except Exception as e:
            snap[m] = f"异常: {e}"

    # channelSamples 中各样本的关键字段
    cs = getattr(d, 'channelSamples', None)
    samples = []
    if cs:
        for ci, ch in enumerate(cs):
            for si, s in enumerate(ch):
                try:
                    samples.append({
                        "ci": ci, "si": si,
                        "data": getattr(s, 'data', None),
                        "rawData": getattr(s, 'rawData', None),
                        "sampleIndex": getattr(s, 'sampleIndex', None),
                    })
                except Exception as e:
                    samples.append({"ci": ci, "si": si, "error": str(e)})
    snap["samples"] = samples
    return snap


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-004 SensorData.to_flatbuffers/from_flatbuffers 往返", flush=True)
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

    # 检查是否支持 to_flatbuffers / from_flatbuffers
    has_to_fb = hasattr(captured_data, 'to_flatbuffers')
    has_from_fb = hasattr(SensorData, 'from_flatbuffers')

    print(f"\n[接口签名检查]", flush=True)
    print(f"  to_flatbuffers: {'存在' if has_to_fb else '不存在'}", flush=True)
    if has_to_fb:
        try:
            sig = inspect.signature(captured_data.to_flatbuffers)
            print(f"    签名: to_flatbuffers{sig}", flush=True)
        except Exception as e:
            print(f"    签名: 无法获取 ({e})", flush=True)

    print(f"  from_flatbuffers: {'存在' if has_from_fb else '不存在'}", flush=True)
    if has_from_fb:
        try:
            sig = inspect.signature(SensorData.from_flatbuffers)
            print(f"    签名: from_flatbuffers{sig}", flush=True)
        except Exception as e:
            print(f"    签名: 无法获取 ({e})", flush=True)

    # 也检查 from_flatbuffers_pooled (供 MISC-FUNC-005 参考)
    has_from_fb_pooled = hasattr(SensorData, 'from_flatbuffers_pooled')
    print(f"  from_flatbuffers_pooled: {'存在' if has_from_fb_pooled else '不存在'}", flush=True)
    if has_from_fb_pooled:
        try:
            sig = inspect.signature(SensorData.from_flatbuffers_pooled)
            print(f"    签名: from_flatbuffers_pooled{sig}", flush=True)
        except Exception as e:
            print(f"    签名: 无法获取 ({e})", flush=True)

    record(results, "SensorData 有 to_flatbuffers()", has_to_fb, "to_flatbuffers 存在", "存在" if has_to_fb else "不存在")
    record(results, "SensorData 有 from_flatbuffers()", has_from_fb, "from_flatbuffers 存在", "存在" if has_from_fb else "不存在")

    if not has_to_fb or not has_from_fb:
        print("[SKIP] 序列化接口不存在，跳过往返测试", flush=True)
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # 序列化前快照
    snap_before = _sensor_data_snapshot(captured_data)
    print(f"\n[序列化前] getDataType={snap_before.get('getDataType')!r}", flush=True)
    print(f"[序列化前] getDeviceMac={snap_before.get('getDeviceMac')!r}", flush=True)
    print(f"[序列化前] getSampleCount={snap_before.get('getSampleCount')!r}", flush=True)
    print(f"[序列化前] samples 数量={len(snap_before.get('samples', []))}", flush=True)

    # to_flatbuffers
    print("\n[序列化] 调用 captured_data.to_flatbuffers() ...", flush=True)
    try:
        fb_bytes = captured_data.to_flatbuffers()
        print(f"[序列化] 返回 bytes, 长度={len(fb_bytes)}", flush=True)
    except Exception as e:
        print(f"[序列化] 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, "to_flatbuffers() 返回 bytes", False, "返回 bytes", f"抛异常 {type(e).__name__}: {e}")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "to_flatbuffers() 返回 bytes", isinstance(fb_bytes, bytes) and len(fb_bytes) > 0,
           "返回非空 bytes", f"长度={len(fb_bytes) if isinstance(fb_bytes, bytes) else '非bytes'}")

    # from_flatbuffers
    print("\n[反序列化] 调用 SensorData.from_flatbuffers(fb_bytes) ...", flush=True)
    try:
        restored = SensorData.from_flatbuffers(fb_bytes)
        print(f"[反序列化] 返回 type={type(restored).__name__}", flush=True)
    except Exception as e:
        print(f"[反序列化] 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, "from_flatbuffers() 返回 SensorData", False, "返回 SensorData", f"抛异常 {type(e).__name__}: {e}")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "from_flatbuffers() 返回 SensorData", isinstance(restored, SensorData),
           "返回 SensorData", f"type={type(restored).__name__}")

    # 往返比对
    snap_after = _sensor_data_snapshot(restored)

    # getDataType 一致
    dt_before = snap_before.get("getDataType")
    dt_after = snap_after.get("getDataType")
    dt_match = (dt_before == dt_after)
    print(f"\n[比对] getDataType: before={dt_before!r} after={dt_after!r} 一致={dt_match}", flush=True)
    record(results, "往返后 getDataType 一致", dt_match,
           "序列化前后 getDataType 相同", f"before={dt_before!r} after={dt_after!r}")

    # getDeviceMac 一致
    mac_before = snap_before.get("getDeviceMac")
    mac_after = snap_after.get("getDeviceMac")
    mac_match = (mac_before == mac_after)
    print(f"[比对] getDeviceMac: before={mac_before!r} after={mac_after!r} 一致={mac_match}", flush=True)
    record(results, "往返后 getDeviceMac 一致", mac_match,
           "序列化前后 getDeviceMac 相同", f"before={mac_before!r} after={mac_after!r}")

    # getSampleCount 一致
    sc_before = snap_before.get("getSampleCount")
    sc_after = snap_after.get("getSampleCount")
    sc_match = (sc_before == sc_after)
    print(f"[比对] getSampleCount: before={sc_before!r} after={sc_after!r} 一致={sc_match}", flush=True)
    record(results, "往返后 getSampleCount 一致", sc_match,
           "序列化前后 getSampleCount 相同", f"before={sc_before!r} after={sc_after!r}")

    # 样本级比对
    samples_before = snap_before.get("samples", [])
    samples_after = snap_after.get("samples", [])
    sample_count_match = (len(samples_before) == len(samples_after))
    print(f"[比对] 样本数: before={len(samples_before)} after={len(samples_after)} 一致={sample_count_match}", flush=True)
    record(results, "往返后样本数一致", sample_count_match,
           "序列化前后样本数相同", f"before={len(samples_before)} after={len(samples_after)}")

    if sample_count_match and len(samples_before) > 0:
        all_sample_match = True
        for i in range(min(len(samples_before), len(samples_after))):
            sb = samples_before[i]
            sa = samples_after[i]
            for f in SAMPLE_FIELDS:
                if sb.get(f) != sa.get(f):
                    print(f"[比对] 样本[{i}].{f}: before={sb.get(f)!r} after={sa.get(f)!r} 不一致!", flush=True)
                    all_sample_match = False
        record(results, "往返后各样本 data/rawData/sampleIndex 一致", all_sample_match,
               "所有样本关键字段一致", "全部一致" if all_sample_match else "存在不一致")
    else:
        record(results, "往返后各样本 data/rawData/sampleIndex 一致", None,
               "所有样本关键字段一致", "样本不足，无法比对")

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
        elif status == "SKIP":
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status == "FAIL":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    asyncio.run(main_async())