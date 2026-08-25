# -*- coding: utf-8 -*-
"""ASYNC-FUNC-011：asyncMultiStartDataNotification 同时起流多台设备，返回 {mac: bool} 且皆为 True。

对应用例：09_异步接口.md -> ASYNC-FUNC-011
可自动化：auto（需 TARGET_IDENTITY 中配置 ≥2 台设备 identity）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 环境中 ≥2 台设备开机且在范围内
  - config.py 中 TARGET_IDENTITY 用逗号列出 ≥2 个 identity

流程：
  1) 确认 ≥2 台设备开机 -> 按回车
  2) scan 匹配 common.TARGET_IDENTITIES 中所有设备（需 ≥2 台，否则 SKIP）
  3) 对每台 requireSensor -> asyncConnect -> 等待 Ready -> asyncInit
  4) 检测型号：同型号默认参数，混型号放宽；await asyncMultiStartDataNotification([s1, s2]) -> 返回 {mac: bool}
  5) 检查返回字典中所有值皆为 True
  6) 检查所有传感器 isDataTransfering==True
  7) 清理：asyncMultiStopDataNotification + asyncDisconnect 所有
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

READY_TIMEOUT = 15

from common import record, _identity_of, async_scan_and_match_all


def _tag_of(d):
    """设备短标识：优先广播名后四位，其次地址。"""
    name = getattr(d, 'Name', '') or ''
    ident = _identity_of(name)
    return ident or (getattr(d, 'Address', '') or '?').upper()


async def main_async():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("ASYNC-FUNC-011 asyncMultiStartDataNotification 多台设备同时起流", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 环境中 ≥2 台设备开机且在范围内", flush=True)
    print(f"  - config.py 中 TARGET_IDENTITY = {config.TARGET_IDENTITY!r}（{len(common.TARGET_IDENTITIES)} 个目标）", flush=True)

    input("\n>>> [人工操作] 请确认 ≥2 台设备已【开机】且在范围内，"
          "且 config.py 中 TARGET_IDENTITY 已列出 ≥2 个 identity，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    targets, devices = await async_scan_and_match_all(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, required=2)
    if len(targets) < 2:
        print(f"\n[SKIP] 需要 ≥2 台设备，当前仅匹配到 {len(targets)} 台", flush=True)
        record(results, "环境中存在 ≥2 台目标设备", None,
               "scan 匹配到 ≥2 台目标设备", f"仅 {len(targets)} 台")

        print("\n" + "=" * 60, flush=True)
        print("测试结果汇总", flush=True)
        print("=" * 60, flush=True)
        for rname, status, expect, actual in results:
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    record(results, "环境中存在 ≥2 台目标设备", True,
           "scan 匹配到 ≥2 台目标设备", f"{len(targets)} 台")

    # requireSensor + connect + Ready + init 每台设备
    sensors = []  # [(tag, sensor)]
    for tid, d in targets:
        tag = _tag_of(d)
        sensor = ctrl.requireSensor(d)
        ok = isinstance(sensor, SensorProfile)
        record(results, f"[{tag}] requireSensor 返回 SensorProfile", ok,
               "返回 SensorProfile", f"返回 {type(sensor).__name__}")
        if not ok:
            print(f"[FAIL] [{tag}] requireSensor 返回 {sensor!r}", flush=True)
            continue

        # connect
        print(f"\n[连接] [{tag}] await SensorProfile.asyncConnect() ...", flush=True)
        try:
            conn_ok = await sensor.asyncConnect()
        except Exception as e:
            conn_ok = False
            print(f"[连接] [{tag}] 抛异常 {type(e).__name__}: {e}", flush=True)
        print(f"[连接] [{tag}] asyncConnect() -> {conn_ok}", flush=True)
        record(results, f"[{tag}] asyncConnect 返回 True", conn_ok is True,
               "asyncConnect() 返回 True", f"asyncConnect() -> {conn_ok}")

        # 等待 Ready
        t0 = time.time()
        while time.time() - t0 < READY_TIMEOUT and sensor.deviceState != DeviceStateEx.Ready:
            await asyncio.sleep(0.2)
        ready = (sensor.deviceState == DeviceStateEx.Ready)
        print(f"[{tag}] deviceState = {sensor.deviceState}", flush=True)
        record(results, f"[{tag}] 到达 Ready", ready,
               "deviceState==Ready", f"deviceState={sensor.deviceState}")

        if not ready:
            print(f"[FAIL] [{tag}] 未到达 Ready，跳过该设备", flush=True)
            continue

        # init
        print(f"[init] [{tag}] await SensorProfile.asyncInit(20, 1000) ...", flush=True)
        try:
            init_ok = await sensor.asyncInit(20, 1000)
        except Exception as e:
            init_ok = False
            print(f"[init] [{tag}] 抛异常 {type(e).__name__}: {e}", flush=True)
        print(f"[init] [{tag}] asyncInit(20, 1000) -> {init_ok}", flush=True)
        record(results, f"[{tag}] asyncInit 返回 True", init_ok is True,
               "asyncInit(20, 1000) 返回 True", f"asyncInit() -> {init_ok}")

        sensors.append((tag, sensor))

    if len(sensors) < 2:
        print(f"\n[SKIP] 成功初始化设备数不足 2 台（实际 {len(sensors)} 台），无法测试 MultiStart", flush=True)
        # 清理
        for tag, sensor in sensors:
            try:
                await sensor.asyncDisconnect()
            except Exception:
                pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    # ---- 判断是否混型号：混型号需放宽对齐参数（参见 05_多设备同步.md MULTI-FUNC-006）----
    model_names = set()
    for tag, sensor in sensors:
        try:
            info = sensor.getDeviceInfo()
        except Exception as e:
            info = None
            print(f"[型号] [{tag}] getDeviceInfo() 抛异常 {type(e).__name__}: {e}", flush=True)
        model_names.add(getattr(info, 'ModelName', None) if info else None)
    same_model = (len(model_names) == 1 and None not in model_names)
    model_txt = sorted(str(m) for m in model_names if m) or ['<未知>']
    print(f"[型号] 检测到 {len(model_names)} 种型号: {model_txt}，same_model={same_model}", flush=True)

    # ---- asyncMultiStartDataNotification ----
    sensor_list = [s for _, s in sensors]
    tag_list = [t for t, _ in sensors]
    if same_model:
        print(f"\n[异步多起流] await ctrl.asyncMultiStartDataNotification([{', '.join(tag_list)}]) ...", flush=True)
        try:
            multi_start_ret = await ctrl.asyncMultiStartDataNotification(sensor_list)
        except Exception as e:
            multi_start_ret = None
            print(f"[异步多起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    else:
        print(f"\n[异步多起流] 混型号，放宽参数 timeout=60, maxDelayDispersionMs=-1, maxAttempts=5 ...", flush=True)
        try:
            multi_start_ret = await ctrl.asyncMultiStartDataNotification(
                sensor_list, timeout=60.0, maxDelayDispersionMs=-1, maxAttempts=5)
        except Exception as e:
            multi_start_ret = None
            print(f"[异步多起流] 抛异常 {type(e).__name__}: {e}", flush=True)

    print(f"[异步多起流] asyncMultiStartDataNotification() -> {multi_start_ret!r}", flush=True)

    is_dict = isinstance(multi_start_ret, dict)
    record(results, "asyncMultiStartDataNotification 返回 dict", is_dict,
           "返回类型为 dict", f"返回 {type(multi_start_ret).__name__}")

    if is_dict:
        all_true = all(v is True for v in multi_start_ret.values())
        record(results, "asyncMultiStartDataNotification 返回所有值皆为 True", all_true,
               "所有 mac 对应值均为 True", f"返回 {multi_start_ret}")

        # 检查每台设备 isDataTransfering
        for tag, sensor in sensors:
            addr = (getattr(sensor, 'mac', None) or getattr(sensor, 'address', None) or '').upper()
            transferring = sensor.isDataTransfering
            print(f"[检查] [{tag}] isDataTransfering = {transferring}", flush=True)
            record(results, f"[{tag}] 起流后 isDataTransfering==True", transferring is True,
                   "isDataTransfering == True", f"isDataTransfering == {transferring}")

    # 清理：asyncMultiStopDataNotification
    print(f"\n[异步多停流] await ctrl.asyncMultiStopDataNotification([{', '.join(tag_list)}]) ...", flush=True)
    try:
        await ctrl.asyncMultiStopDataNotification(sensor_list)
    except Exception as e:
        print(f"[异步多停流] 抛异常 {type(e).__name__}: {e}", flush=True)

    # 断开所有
    for tag, sensor in sensors:
        try:
            await sensor.asyncDisconnect()
        except Exception as e:
            print(f"[断开] [{tag}] 抛异常 {type(e).__name__}: {e}", flush=True)

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