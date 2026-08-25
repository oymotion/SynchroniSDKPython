# -*- coding: utf-8 -*-
"""ASYNC-FUNC-005：asyncSetParam 设置 NTF_EEG 生效性（起流验证）。

对应用例：09_异步接口.md -> ASYNC-FUNC-005
可自动化：auto（需待测设备上电在范围内）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor
  3) await sensor.asyncConnect() -> 到达 Ready
  4) await sensor.asyncInit(20, 1000)
  5) await sensor.asyncSetParam("NTF_EEG", "ON") -> 断言返回 "OK"
  6) 起流采集 -> 断言收到 NTF_EEG 数据（ON 生效）
  7) await sensor.asyncSetParam("NTF_EEG", "OFF") -> 断言返回 "OK"
  8) 起流采集 -> 断言无 NTF_EEG 数据（OFF 生效）
  9) await sensor.asyncSetParam("NTF_EEG", "ON") 再次设置 -> 断言返回 "OK"
 10) 起流采集 -> 断言再次收到 NTF_EEG 数据（OFF->ON 可逆）
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
import common
from common import record, scan_and_match, async_scan_and_match


COLLECT_SECONDS = 3


class EegResult:
    """统计 NTF_EEG 数据流的批数与样本数。"""

    def __init__(self):
        self.batches = 0
        self.eeg_samples = 0


def make_eeg_on_data(result):
    def on_data(sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            if d.getDataType() != DataType.NTF_EEG:
                continue
            result.batches += 1
            cs = getattr(d, 'channelSamples', None)
            n = 0
            if cs:
                try:
                    n = sum(len(ch) for ch in cs)
                except TypeError:
                    n = len(cs)
            result.eeg_samples += n
    return on_data


async def _start_collect_stop(sensor, seconds):
    """起流 -> 采集 seconds 秒 -> 停流，返回 NTF_EEG 样本数（起流失败返回 -1）。"""
    result = EegResult()
    sensor.onDataCallback = make_eeg_on_data(result)
    start_ok = False
    try:
        start_ok = await sensor.asyncStartDataNotification()
    except Exception as e:
        print(f"[起流] asyncStartDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    if start_ok is not True:
        print(f"[起流] asyncStartDataNotification 返回 {start_ok!r}", flush=True)
        sensor.onDataCallback = None
        return -1
    await asyncio.sleep(seconds)
    try:
        await sensor.asyncStopDataNotification()
    except Exception as e:
        print(f"[停流] asyncStopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    sensor.onDataCallback = None
    return result.eeg_samples


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("ASYNC-FUNC-005 asyncSetParam 设置 NTF_EEG 生效性（起流验证）", flush=True)
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
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}（config.TARGET_IDENTITY = {config.TARGET_IDENTITY!r}）", flush=True)
    target, devices = await async_scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    # 打印扫描到的设备列表
    scanned = [(getattr(d, 'Name', '?'), getattr(d, 'Address', '?')) for d in (devices or [])]
    print(f"[扫描] 扫描到 {len(scanned)} 台设备: {scanned}", flush=True)


    if target is None:
        print(f"[FAIL] 未匹配到目标设备（目标 identity: {common.TARGET_IDENTITIES}，扫描到: {scanned}）", flush=True)
        print(f"[提示] 请检查 config.py 中 TARGET_IDENTITY 是否设置为正确的设备 identity。", flush=True)
        print(f"[提示] 当前 DEVICES 配置中可用的 identity: {[c.get('identity') for c in config.DEVICES]}", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 匹配到目标设备: {name} {addr}", flush=True)
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

    # ---- asyncSetParam ON ----
    print("\n[异步设置-ON] await sensor.asyncSetParam('NTF_EEG', 'ON') ...", flush=True)
    try:
        r_on = await sensor.asyncSetParam("NTF_EEG", "ON")
    except Exception as e:
        r_on = f"抛异常 {type(e).__name__}: {e}"
        print(f"[异步设置-ON] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步设置-ON] sensor.asyncSetParam('NTF_EEG', 'ON') -> {r_on!r}", flush=True)
    record(results, "asyncSetParam('NTF_EEG', 'ON') 返回 'OK'", r_on == "OK",
           "asyncSetParam 返回 'OK'", f"asyncSetParam() -> {r_on!r}")

    # 起流验证 ON：设置 ON 后应能收到 EEG 数据
    print(f"\n[起流验证-ON] 起流采集 {COLLECT_SECONDS}s 验证 NTF_EEG=ON 生效 ...", flush=True)
    eeg_on = await _start_collect_stop(sensor, COLLECT_SECONDS)
    print(f"[起流验证-ON] NTF_EEG 样本数 = {eeg_on}", flush=True)
    record(results, "NTF_EEG=ON 起流后收到 EEG 数据", eeg_on > 0,
           "起流后 NTF_EEG 样本数 > 0", f"NTF_EEG 样本数 = {eeg_on}")

    # ---- asyncSetParam OFF ----
    print("\n[异步设置-OFF] await sensor.asyncSetParam('NTF_EEG', 'OFF') ...", flush=True)
    try:
        r_off = await sensor.asyncSetParam("NTF_EEG", "OFF")
    except Exception as e:
        r_off = f"抛异常 {type(e).__name__}: {e}"
        print(f"[异步设置-OFF] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[异步设置-OFF] sensor.asyncSetParam('NTF_EEG', 'OFF') -> {r_off!r}", flush=True)
    record(results, "asyncSetParam('NTF_EEG', 'OFF') 返回 'OK'", r_off == "OK",
           "asyncSetParam 返回 'OK'", f"asyncSetParam() -> {r_off!r}")

    # 起流验证 OFF：设置 OFF 后应无 EEG 数据
    print(f"\n[起流验证-OFF] 起流采集 {COLLECT_SECONDS}s 验证 NTF_EEG=OFF 生效 ...", flush=True)
    eeg_off = await _start_collect_stop(sensor, COLLECT_SECONDS)
    print(f"[起流验证-OFF] NTF_EEG 样本数 = {eeg_off}", flush=True)
    record(results, "NTF_EEG=OFF 起流后无 EEG 数据", eeg_off == 0,
           "起流后 NTF_EEG 样本数 == 0", f"NTF_EEG 样本数 = {eeg_off}")

    # ---- 再次设置 ON 并起流验证：OFF -> ON 可逆 ----
    print("\n[再次设置-ON] await sensor.asyncSetParam('NTF_EEG', 'ON') ...", flush=True)
    try:
        r_sync = await sensor.asyncSetParam("NTF_EEG", "ON")
    except Exception as e:
        r_sync = f"抛异常 {type(e).__name__}: {e}"
        print(f"[再次设置-ON] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[再次设置-ON] sensor.asyncSetParam('NTF_EEG', 'ON') -> {r_sync!r}", flush=True)
    record(results, "asyncSetParam('NTF_EEG', 'ON') 再次设置返回 'OK'", r_sync == "OK",
           "asyncSetParam 返回 'OK'", f"asyncSetParam() -> {r_sync!r}")

    print(f"\n[起流验证-再次ON] 起流采集 {COLLECT_SECONDS}s 验证 OFF->ON 可逆 ...", flush=True)
    eeg_on2 = await _start_collect_stop(sensor, COLLECT_SECONDS)
    print(f"[起流验证-再次ON] NTF_EEG 样本数 = {eeg_on2}", flush=True)
    record(results, "NTF_EEG 再次 ON 后收到 EEG 数据", eeg_on2 > 0,
           "起流后 NTF_EEG 样本数 > 0", f"NTF_EEG 样本数 = {eeg_on2}")

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