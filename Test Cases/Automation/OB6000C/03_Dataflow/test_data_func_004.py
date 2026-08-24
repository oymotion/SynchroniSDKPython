# -*- coding: utf-8 -*-
"""DATA-FUNC-004：stopDataNotification 后停止。

对应用例：03_数据流.md -> DATA-FUNC-004
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready
  2) init -> startDataNotification 起流，确认收到数据
  3) stopDataNotification 停流
  4) 断言 isDataTransfering==False，且回调停止（停流后批数不再增长）

可证伪点：
  - 停流后 isDataTransfering 仍 True -> FAIL（标志位未复位）
  - 停流后回调仍在增长 -> FAIL（停流不干净，残留数据流）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config

COLLECT_SECONDS = 3  # 起流后采集时长（秒），确认有数据
DRAIN_WINDOW = 2     # 停流后刹车期（秒），等在途数据排空
STABLE_WINDOW = 3    # 刹车后稳定观察期（秒），确认批数不再增长

from common import record, scan_and_match


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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-004 stopDataNotification 后停止", flush=True)
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
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含启用的目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含启用的目标设备", f"匹配到 {name} {addr}")

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # connect
    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
        connect_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {connect_txt}  state={sensor.deviceState}", flush=True)
    record(results, "SensorProfile.connect 返回 True", ok is True,
           "connect() 返回 True", f"connect() -> {connect_txt}")

    # 到达 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    ready = (sensor.deviceState == DeviceStateEx.Ready)
    record(results, "connect 后到达 Ready", ready, "deviceState==Ready", f"state={sensor.deviceState}")

    if not ready:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # init
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    # 起流
    result = DataResult()
    sensor.onDataCallback = make_on_data(result)

    print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    transferring = sensor.isDataTransfering
    record(results, "起流后 isDataTransfering==True", transferring is True,
           "isDataTransfering == True", f"isDataTransfering == {transferring}")

    # 采集，确认收到数据
    print(f"\n[采集] 等待 {COLLECT_SECONDS}s 确认收到数据 ...", flush=True)
    time.sleep(COLLECT_SECONDS)
    print(f"[采集] 批数={result.batches} 样本数={result.total_samples}", flush=True)
    record(results, "起流后收到数据（样本数>0）", result.total_samples > 0,
           "onDataCallback 收到样本", f"批数={result.batches} 样本数={result.total_samples}")

    # 停流
    before_stop = result.batches
    print("\n[停流] SensorProfile.stopDataNotification() ...", flush=True)
    try:
        stop_ret = sensor.stopDataNotification()
        stop_txt = f"返回 {stop_ret}"
    except Exception as e:
        stop_ret = None
        stop_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[停流] SensorProfile.stopDataNotification() -> {stop_txt}", flush=True)
    record(results, "SensorProfile.stopDataNotification 返回 True", stop_ret is True,
           "stopDataNotification() 返回 True", f"stopDataNotification() -> {stop_txt}")

    # 停流后观察：先刹车期排空在途数据，再稳定期确认批数不再增长
    print(f"\n[观察] 刹车期 {DRAIN_WINDOW}s 等在途数据排空 ...", flush=True)
    time.sleep(DRAIN_WINDOW)
    baseline = result.batches
    print(f"[观察] 刹车期后批数={baseline}，稳定期 {STABLE_WINDOW}s 确认不再增长 ...", flush=True)
    time.sleep(STABLE_WINDOW)
    final = result.batches
    transferring_after = sensor.isDataTransfering

    growth = final - baseline
    print(f"[观察] 停流前批数={before_stop} 刹车后批数={baseline} 稳定期后批数={final} "
          f"增长={growth} isDataTransfering={transferring_after}", flush=True)
    record(results, "停流后 isDataTransfering==False", transferring_after is False,
           "isDataTransfering == False", f"isDataTransfering == {transferring_after}")
    record(results, "停流后回调停止（稳定期批数不再增长）", growth == 0,
           "刹车期排空后，稳定期内批数不再增长",
           f"停流前批数={before_stop} 刹车后批数={baseline} 稳定期后批数={final} 增长={growth}")

    # 清理
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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
    main()
