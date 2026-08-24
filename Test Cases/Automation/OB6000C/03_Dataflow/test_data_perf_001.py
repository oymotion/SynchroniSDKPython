# -*- coding: utf-8 -*-
"""DATA-PERF-001：实测采样率与标称偏差 ≤ 容差。

对应用例：03_数据流.md -> DATA-PERF-001
可自动化：auto（设备上电、在范围内为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EEG", "ON") 起 EEG 流
  3) startDataNotification 后采集窗口内统计样本，实测采样率与 getSampleRate() 比较
  4) 校验：偏差 ≤ 容差（默认 ±5%）

说明：
  实测采样率用"固定窗口计时"计算：从首批数据到达起，精确采集 PERF_COLLECT_SECONDS
  秒，用固定墙钟时长做分母（而非首末批到达时间），彻底排除起流前延迟和停流空转：
      实测采样率 = 每通道累计样本数 / PERF_COLLECT_SECONDS
  EEG 采样率（getSampleRate() 标称，多通道），延长采集时长可进一步确认收敛趋势。
  采样率统计与是否佩戴无关（佩戴只影响信号内容，不影响采样速率）。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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

SAMPLE_RATE_TOLERANCE = 0.05  # 采样率容差 ±5%
MIN_SAMPLES_FOR_RATE = 100    # 计算采样率所需最小样本数（保证统计精度）
PERF_COLLECT_SECONDS = 30     # 固定窗口采集时长（秒），用于实测采样率统计


class RateCollector:
    """记录首末批次到达时间与每通道累计样本数，用于计算实测采样率。"""

    def __init__(self):
        self.first_ts = None     # 首批到达时间
        self.last_ts = None      # 末批到达时间
        self.total_samples = 0   # 每通道累计样本数（时间维度）
        self.batches = 0
        self.first_batch = None  # 第一个非空 SensorData（用于读取 getSampleRate）
        self.min_sample_index = None  # 通道0 sampleIndex 最小值（交叉验证样本唯一性）
        self.max_sample_index = None  # 通道0 sampleIndex 最大值

    def on_data(self, sensor, data):
        now = time.time()
        if self.first_ts is None:
            self.first_ts = now
        self.last_ts = now
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.batches += 1
            cs = getattr(d, 'channelSamples', None)
            if cs:
                try:
                    n_ch = len(cs)
                    self.total_samples += len(cs[0]) if n_ch else 0
                    # 记录通道0样本的 sampleIndex 首末（交叉验证样本唯一性）
                    if n_ch:
                        for s in cs[0]:
                            idx = getattr(s, 'sampleIndex', None)
                            if idx is None:
                                continue
                            if self.min_sample_index is None or idx < self.min_sample_index:
                                self.min_sample_index = idx
                            if self.max_sample_index is None or idx > self.max_sample_index:
                                self.max_sample_index = idx
                except TypeError:
                    self.total_samples += len(cs)
                if self.first_batch is None:
                    self.first_batch = d


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-PERF-001 实测采样率与标称偏差 <= 容差", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
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
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
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

    # 起 EEG 流
    print("\n[起流] SensorProfile.setParam('NTF_EEG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EEG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EEG', 'ON') -> {p_txt}", flush=True)

    collector = RateCollector()
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

    # 采集窗口（固定窗口计时：从首批到达开始精确采集 PERF_COLLECT_SECONDS）
    print("\n[采集] 等待首批数据到达 ...", flush=True)
    t_wait0 = time.time()
    while collector.first_ts is None and time.time() - t_wait0 < 10:
        time.sleep(0.05)

    if collector.first_ts is None:
        print("[采集] 10s 内未收到任何数据", flush=True)
        window_duration = None
    else:
        t_end = collector.first_ts + PERF_COLLECT_SECONDS
        print(f"[采集] 首批已到达，固定窗口采集 {PERF_COLLECT_SECONDS}s ...", flush=True)
        while time.time() < t_end:
            time.sleep(0.05)
        window_duration = PERF_COLLECT_SECONDS

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.setParam("NTF_EEG", "OFF")
    except Exception:
        pass

    # 读取标称采样率（getSampleRate 是 SensorData 的方法，非 SensorProfile）
    if collector.first_batch is None:
        nominal_rate = None
        print("[读取] 未收到数据，无法读取标称采样率", flush=True)
    else:
        try:
            nominal_rate = float(collector.first_batch.getSampleRate())
        except Exception as e:
            nominal_rate = None
            print(f"[读取] SensorData.getSampleRate() 抛异常 {type(e).__name__}: {e}", flush=True)

    print(f"\n[统计] 批次数={collector.batches} 每通道累计样本数={collector.total_samples}", flush=True)
    print(f"[统计] 固定窗口时长={window_duration if window_duration is not None else 'N/A'}s", flush=True)
    print(f"[标称] getSampleRate() = {nominal_rate}", flush=True)

    # 判定 1：采集到足够样本
    record(results, "采集到足够样本（>= %d）" % MIN_SAMPLES_FOR_RATE,
           collector.total_samples >= MIN_SAMPLES_FOR_RATE,
           f"每通道累计样本数 >= {MIN_SAMPLES_FOR_RATE}",
           f"每通道累计样本数={collector.total_samples}")

    # 判定 2：收到数据（首批已到达）
    record(results, "收到数据（首批已到达）", collector.first_ts is not None,
           "首批数据已到达", "未收到数据" if collector.first_ts is None else "已收到数据")

    # 判定 3：实测采样率偏差 <= 容差
    if window_duration is None or collector.total_samples <= 0:
        record(results, "实测采样率偏差 <= 容差", False,
               f"|实测-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.0%}",
               "未收到任何数据，无法计算实测采样率")
    else:
        measured_rate = collector.total_samples / window_duration

        if nominal_rate is None:
            record(results, "实测采样率偏差 <= 容差", False,
                   f"|实测-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.0%}",
                   f"实测采样率={measured_rate:.1f}Hz 但 getSampleRate() 抛异常，无法比较")
        elif nominal_rate <= 0:
            record(results, "实测采样率偏差 <= 容差", False,
                   f"|实测-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.0%}",
                   f"标称采样率非法（{nominal_rate}），无法比较")
        else:
            deviation = abs(measured_rate - nominal_rate) / nominal_rate
            record(results, "实测采样率偏差 <= 容差", deviation <= SAMPLE_RATE_TOLERANCE,
                   f"|实测-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.0%}",
                   f"实测={measured_rate:.1f}Hz 标称={nominal_rate:.1f}Hz 偏差={deviation:.2%}")

    # 交叉验证：sampleIndex 增量 vs channelSamples 长度累加
    if window_duration and collector.min_sample_index is not None and collector.max_sample_index is not None:
        index_span = collector.max_sample_index - collector.min_sample_index + 1
        index_rate = index_span / window_duration
        len_rate = collector.total_samples / window_duration
        print(f"\n[交叉验证] sampleIndex 范围 = [{collector.min_sample_index}, {collector.max_sample_index}]", flush=True)
        print(f"[交叉验证] sampleIndex 增量 = {index_span}（唯一样本数）", flush=True)
        print(f"[交叉验证] channelSamples 长度累加 = {collector.total_samples}（投递样本数）", flush=True)
        print(f"[交叉验证] 按 sampleIndex 换算采样率 = {index_rate:.1f}Hz", flush=True)
        print(f"[交叉验证] 按 channelSamples 换算采样率 = {len_rate:.1f}Hz", flush=True)
        if index_span > 0:
            diff_ratio = abs(collector.total_samples - index_span) / index_span
            if diff_ratio < 0.01:
                print(f"[交叉验证] 结论：两者差异 {diff_ratio:.2%}，样本唯一 → 实测偏高为真实数据速率（疑似重复投递）", flush=True)
            else:
                print(f"[交叉验证] 结论：两者差异 {diff_ratio:.2%}，channelSamples 存在重复，实际唯一样本 ≈ {index_span}", flush=True)
    else:
        print("\n[交叉验证] 未能读取 sampleIndex，跳过", flush=True)

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
