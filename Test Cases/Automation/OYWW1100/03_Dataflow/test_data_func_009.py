# -*- coding: utf-8 -*-
"""DATA-FUNC-009：每批样本数 ≈ packageSampleCount。

对应用例：03_数据流.md -> DATA-FUNC-009
可自动化：auto（设备上电、在范围内且已佩戴为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init(config.PACKAGE_SAMPLE_COUNT, ...)
  2) setParam("NTF_EMG", "ON") 起 EMG 流
  3) startDataNotification 后采集窗口内收集所有批次
  4) 统计每批"每通道样本数"（len(channelSamples[0])），校验：
     - 多数批 == packageSampleCount（满批占比 >= 80%）
     - 无超批（每批样本数 <= packageSampleCount）
     - 允许首尾边界批不足

说明：
  packageSampleCount 是 init 时设定的"每包样本数"（时间维度样本点数）。
  一批 SensorData 的每通道样本数应约等于该值（多数批精确相等）。
  允许边界批不足（如停流时刻截断的最后一批）。
  EMG 采样率 500Hz、packageSampleCount=20，每秒约 25 批，5s 内上百批，统计充分。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内，且已佩戴（电极接触皮肤）
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match


class BatchSizeCollector:
    """收集所有批次，记录每批的"每通道样本数"（len(channelSamples[0])）。"""

    def __init__(self):
        self.batch_sizes = []  # 每批的每通道样本数
        self.total_samples = 0

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            cs = getattr(d, 'channelSamples', None)
            n_s = 0
            if cs:
                try:
                    n_ch = len(cs)
                    n_s = len(cs[0]) if n_ch else 0
                except TypeError:
                    n_s = len(cs)
            self.batch_sizes.append(n_s)
            self.total_samples += n_s


def check_batch_sizes(batch_sizes, package_sample_count, results):
    """校验每批样本数是否 ≈ packageSampleCount。"""

    def add(name, ok, expect, actual):
        record(results, name, ok, expect, actual)

    total = len(batch_sizes)
    full = sum(1 for n in batch_sizes if n == package_sample_count)
    over = sum(1 for n in batch_sizes if n > package_sample_count)
    under = sum(1 for n in batch_sizes if 0 < n < package_sample_count)
    empty = sum(1 for n in batch_sizes if n == 0)
    ratio = full / total if total else 0

    # 判定 1：收到足够批次
    add("采集窗口内收到足够批次（>= 5 批）", total >= 5,
        "收到 >= 5 批 SensorData", f"批数={total}")

    # 判定 2：多数批满批
    add("多数批样本数 == packageSampleCount（满批占比 >= 80%）", total > 0 and ratio >= 0.8,
        f"满批（=={package_sample_count}）占比 >= 80%",
        f"总批数={total} 满批={full} 不足批={under} 空批={empty} 超批={over} 满批占比={ratio:.0%}")

    # 判定 3：无超批
    add("无超批（每批样本数 <= packageSampleCount）", over == 0,
        f"每批样本数 <= {package_sample_count}",
        f"超批={over}" if over else "无超批")


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-009 每批样本数 ≈ packageSampleCount", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内，且已佩戴（电极接触皮肤）", flush=True)
    print(f"  - init packageSampleCount = {config.PACKAGE_SAMPLE_COUNT}", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】、在范围内且已【佩戴】，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到目标设备（OYWW1100/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OYWW1100", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OYWW1100", f"匹配到 {name} {addr}")

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

    # 起 EMG 流
    print("\n[起流] SensorProfile.setParam('NTF_EMG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EMG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EMG', 'ON') -> {p_txt}", flush=True)

    collector = BatchSizeCollector()
    sensor.onDataCallback = collector.on_data

    print("[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    # 采集窗口，收集所有批次
    print(f"\n[采集] 等待 {config.COLLECT_SECONDS}s 观察 onDataCallback ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    print(f"[采集] 收到批数={len(collector.batch_sizes)} 每通道样本总数={collector.total_samples}", flush=True)

    # 打印每批样本数分布（供定位）
    if collector.batch_sizes:
        from collections import Counter
        dist = Counter(collector.batch_sizes)
        print(f"[采集] 每批样本数分布（样本数->批数）: {dict(sorted(dist.items()))}", flush=True)

    check_batch_sizes(collector.batch_sizes, config.PACKAGE_SAMPLE_COUNT, results)

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass
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
