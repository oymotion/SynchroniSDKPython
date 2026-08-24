# -*- coding: utf-8 -*-
"""MISC-FUNC-005：SensorData.from_flatbuffers_pooled 池化。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-005
可自动化：auto（需待测设备上电在范围内，起流获取 SensorData 对象）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OYWW1100 -> requireSensor -> connect -> init -> startDataNotification
  3) 从 onDataCallback 获取一个 SensorData 对象
  4) to_flatbuffers() 得 bytes
  5) from_flatbuffers_pooled(bytes) 还原，检查与 from_flatbuffers 一致
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

# 显式导入 SensorDataPool（from sensor import * 可能不包含它）
try:
    from sensor.sensor_data_pool import SensorDataPool
except ImportError:
    try:
        from sensor import SensorDataPool
    except ImportError:
        SensorDataPool = None

COLLECT_SECONDS = 5


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-005 SensorData.from_flatbuffers_pooled 池化", flush=True)
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

    # 检查是否支持序列化接口
    has_to_fb = hasattr(captured_data, 'to_flatbuffers')
    has_from_fb = hasattr(SensorData, 'from_flatbuffers')
    has_from_fb_pooled = hasattr(SensorData, 'from_flatbuffers_pooled')

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

    print(f"  from_flatbuffers_pooled: {'存在' if has_from_fb_pooled else '不存在'}", flush=True)
    if has_from_fb_pooled:
        try:
            sig = inspect.signature(SensorData.from_flatbuffers_pooled)
            print(f"    签名: from_flatbuffers_pooled{sig}", flush=True)
        except Exception as e:
            print(f"    签名: 无法获取 ({e})", flush=True)

    record(results, "SensorData 有 from_flatbuffers_pooled()", has_from_fb_pooled,
           "from_flatbuffers_pooled 存在", "存在" if has_from_fb_pooled else "不存在")

    if not has_to_fb or not has_from_fb or not has_from_fb_pooled:
        print("[SKIP] 序列化接口不全，跳过池化测试", flush=True)
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

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

    # from_flatbuffers（标准）
    print("\n[反序列化-标准] SensorData.from_flatbuffers(fb_bytes) ...", flush=True)
    try:
        restored_std = SensorData.from_flatbuffers(fb_bytes)
        print(f"[反序列化-标准] type={type(restored_std).__name__}", flush=True)
    except Exception as e:
        print(f"[反序列化-标准] 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, "from_flatbuffers() 成功", False, "返回 SensorData", f"抛异常 {type(e).__name__}: {e}")
        try:
            await sensor.asyncDisconnect()
        except Exception:
            pass
        ctrl.terminate()
        return
    record(results, "from_flatbuffers() 成功", isinstance(restored_std, SensorData),
           "返回 SensorData", f"type={type(restored_std).__name__}")

    # from_flatbuffers_pooled
    print("\n[反序列化-池化] SensorData.from_flatbuffers_pooled(fb_bytes) ...", flush=True)

    # 探测需要哪些参数
    try:
        sig = inspect.signature(SensorData.from_flatbuffers_pooled)
        params = list(sig.parameters.keys())
        print(f"[反序列化-池化] 签名: from_flatbuffers_pooled{sig}", flush=True)
        print(f"[反序列化-池化] 参数列表: {params}", flush=True)
    except Exception as e:
        params = None
        print(f"[反序列化-池化] 无法获取签名: {e}", flush=True)

    try:
        # 签名: from_flatbuffers_pooled(buf: bytes, data: SensorData, pool=None)
        # 参数1: buf (bytes)     — 序列化数据
        # 参数2: data (SensorData) — 复用的 SensorData 对象
        # 参数3: pool (optional)  — 对象池，默认 None
        # 传入 captured_data 作为复用对象，不传 pool
        restored_pooled = SensorData.from_flatbuffers_pooled(fb_bytes, captured_data)
        print(f"[反序列化-池化] type={type(restored_pooled).__name__}", flush=True)
        record(results, "from_flatbuffers_pooled() 成功", isinstance(restored_pooled, SensorData),
               "返回 SensorData", f"type={type(restored_pooled).__name__}")
    except Exception as e:
        print(f"[反序列化-池化] 抛异常 {type(e).__name__}: {e}", flush=True)
        record(results, "from_flatbuffers_pooled() 成功", False, "返回 SensorData", f"抛异常 {type(e).__name__}: {e}")
        restored_pooled = None

    # 比对 pooled 与标准结果一致
    if restored_pooled is not None:
        try:
            dt_std = restored_std.getDataType()
            dt_pooled = restored_pooled.getDataType()
            dt_match = (dt_std == dt_pooled)
        except Exception as e:
            dt_match = False
            print(f"[比对] getDataType 抛异常: {e}", flush=True)
        print(f"[比对] getDataType: std={dt_std!r} pooled={dt_pooled!r} 一致={dt_match}", flush=True)
        record(results, "pooled 与标准 from_flatbuffers 的 getDataType 一致", dt_match,
               "两者 getDataType 相同", f"std={dt_std!r} pooled={dt_pooled!r}")

        try:
            sc_std = restored_std.getSampleCount()
            sc_pooled = restored_pooled.getSampleCount()
            sc_match = (sc_std == sc_pooled)
        except Exception as e:
            sc_match = False
            print(f"[比对] getSampleCount 抛异常: {e}", flush=True)
        print(f"[比对] getSampleCount: std={sc_std!r} pooled={sc_pooled!r} 一致={sc_match}", flush=True)
        record(results, "pooled 与标准 from_flatbuffers 的 getSampleCount 一致", sc_match,
               "两者 getSampleCount 相同", f"std={sc_std!r} pooled={sc_pooled!r}")
    else:
        record(results, "pooled 与标准 from_flatbuffers 的 getDataType 一致", False,
               "pooled 反序列化成功", "pooled 反序列化失败，跳过比对")

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