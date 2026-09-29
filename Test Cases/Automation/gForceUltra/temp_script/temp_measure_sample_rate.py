# -*- coding: utf-8 -*-
"""临时脚本：gForceUltra EMG 采样率实测（500 / 1000 各 60s，sampleIndex 去重）。

用途：临时验证 gForceUltra 两档 EMG 采样率的真实数据率，不并入正式用例。

做法：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 显式按 RATES = ["500", "1000"] 逐一 setParam("EMG_SAMPLE_RATE", rate)
  3) 起流后按通道 0 的 sampleIndex 首末跨度去重，得到唯一样本数
  4) 实际采样率 = 唯一样本数 / 采集时长；每档各测 60s

要点：
  - 采样率显式写死在脚本里（500 先测，1000 后测），不做 getParam 列表推导。
  - 测量窗口从「首批 EMG 数据到达」起算，排除起流建立延迟。
  - 去重口径与 DATA-FUNC-011 / perf 脚本一致：unique = max_index - min_index + 1；
    同时保留「投递样本数」做重复投递的交叉观测。

用法：
  python temp_measure_sample_rate.py
  python temp_measure_sample_rate.py 80E5   # 指定 gForceUltra identity

前置条件：
  - 主机蓝牙已开启（或 USB dongle 已就绪）
  - 待测 gForceUltra 上电、在范围内
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import scan_and_match

RATES = ["500", "1000"]        # gForceUltra 两档 EMG 采样率，显式设置
MEASURE_SECONDS = 180           # 每档采集时长（秒）
SETTLE_SECONDS = 2.0           # setParam 后等待设备异步生效的静置时长
READY_TIMEOUT = 15             # 连接后等待 Ready 超时（秒）
FIRST_DATA_TIMEOUT = 10        # 起流后等待首批数据超时（秒）


class RateCollector:
    """统计 NTF_EMG 样本数（sampleIndex 去重），用于计算实际采样率。"""

    def __init__(self):
        self.delivered_samples = 0   # 投递样本数（通道×样本累加，含重复，仅观测）
        self.channel_count = 0
        self.first_ts = None         # 首批 EMG 数据到达时刻
        self.min_sample_index = None  # 通道0 sampleIndex 最小值（跨批单调递增）
        self.max_sample_index = None  # 通道0 sampleIndex 最大值

    def reset(self):
        self.delivered_samples = 0
        self.channel_count = 0
        self.first_ts = None
        self.min_sample_index = None
        self.max_sample_index = None

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            if dt != DataType.NTF_EMG:
                continue
            try:
                n_ch = d.getChannelCount()
                n_smp = d.getSampleCount()
            except Exception:
                n_ch = n_smp = 0
            if n_ch <= 0 or n_smp <= 0:
                continue
            if self.first_ts is None:
                self.first_ts = time.time()
            self.delivered_samples += n_ch * n_smp
            if self.channel_count == 0:
                self.channel_count = n_ch
            # 通道0 sampleIndex 记录全局首末（跨批单调递增，去重）
            for si in range(n_smp):
                try:
                    idx = d.getSampleIndex(0, si)
                except Exception:
                    continue
                if idx is None:
                    continue
                if self.min_sample_index is None or idx < self.min_sample_index:
                    self.min_sample_index = idx
                if self.max_sample_index is None or idx > self.max_sample_index:
                    self.max_sample_index = idx

    @property
    def unique_samples(self):
        """唯一样本数 = sampleIndex 跨度（去重，不含重复投递）。"""
        if self.min_sample_index is None or self.max_sample_index is None:
            return 0
        return self.max_sample_index - self.min_sample_index + 1


def main():
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("临时脚本：gForceUltra EMG 采样率实测（500/1000 各 60s）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"待测采样率（显式）: {RATES}，每档 {MEASURE_SECONDS}s", flush=True)

    # 扫描
    print(f"\n[扫描] 目标 identity: {target_identity or common.TARGET_IDENTITIES} ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return
    print(f"[扫描] 目标设备: {getattr(target, 'Name', '?')} {getattr(target, 'Address', '?')}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    # 连接
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = None
        print(f"[连接] connect 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[连接] connect() -> {ok}  state={sensor.deviceState}", flush=True)
    if ok is not True:
        print("[FAIL] connect 失败", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[FAIL] 未到达 Ready（state={sensor.deviceState}）", flush=True)
        ctrl.terminate()
        return

    # init
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        iret = None
        print(f"[init] init 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[init] init() -> {iret}  hasInited={sensor.hasInited}", flush=True)
    if iret is not True:
        print("[FAIL] init 失败", flush=True)
        ctrl.terminate()
        return

    # 开启 EMG 流开关（gForceUltra 主模态）
    try:
        sensor.setParam("NTF_EMG", "ON")
    except Exception as e:
        print(f"[setParam] NTF_EMG ON 抛异常 {type(e).__name__}: {e}", flush=True)

    collector = RateCollector()
    sensor.onDataCallback = collector.on_data

    rate_summaries = []

    for rate in RATES:
        print("\n" + "=" * 40, flush=True)
        print(f"[测试] EMG_SAMPLE_RATE = {rate}", flush=True)

        # 显式设置采样率
        try:
            r = sensor.setParam("EMG_SAMPLE_RATE", rate)
        except Exception as e:
            r = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] EMG_SAMPLE_RATE={rate} -> {r!r}", flush=True)
        if r != "OK":
            rate_summaries.append((rate, None, f"setParam 返回 {r!r}"))
            continue

        # 读回确认
        try:
            cur = sensor.getParam("EMG_SAMPLE_RATE")
            print(f"[getParam] EMG_SAMPLE_RATE = {cur!r}", flush=True)
        except Exception as e:
            print(f"[getParam] EMG_SAMPLE_RATE 抛异常 {type(e).__name__}: {e}", flush=True)

        # 静置等待设备异步生效
        print(f"[等待] 静置 {SETTLE_SECONDS}s 等待采样率生效 ...", flush=True)
        time.sleep(SETTLE_SECONDS)

        # 起流
        collector.reset()
        try:
            sret = sensor.startDataNotification()
        except Exception as e:
            sret = None
            print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)
        print(f"[起流] startDataNotification() -> {sret}", flush=True)
        if sret is not True:
            rate_summaries.append((rate, None, "startDataNotification 失败"))
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            continue

        # 等首批数据，测量窗口从首批起算
        print("[采集] 等待首批 EMG 数据到达 ...", flush=True)
        t_wait0 = time.time()
        while collector.first_ts is None and time.time() - t_wait0 < FIRST_DATA_TIMEOUT:
            time.sleep(0.05)

        if collector.first_ts is None:
            print(f"[采集] {FIRST_DATA_TIMEOUT}s 内未收到 EMG 数据", flush=True)
            collect_duration = 0
        else:
            t_end = collector.first_ts + MEASURE_SECONDS
            print(f"[采集] 首批已到达，从首批起精确采集 {MEASURE_SECONDS}s ...", flush=True)
            while time.time() < t_end:
                time.sleep(0.05)
            collect_duration = time.time() - collector.first_ts

        try:
            sensor.stopDataNotification()
        except Exception as e:
            print(f"[停流] 抛异常 {type(e).__name__}: {e}", flush=True)

        unique = collector.unique_samples
        actual_rate = (unique / collect_duration) if (collect_duration > 0 and unique > 0) else 0
        expected_rate = int(rate)
        tolerance = expected_rate * 0.10
        rate_ok = abs(actual_rate - expected_rate) <= tolerance

        print(f"[结果] 期望={expected_rate}Hz, 实际≈{actual_rate:.1f}Hz "
              f"(唯一样本={unique} 投递样本={collector.delivered_samples} "
              f"通道={collector.channel_count} 时长={collect_duration:.1f}s)", flush=True)
        rate_summaries.append((rate, rate_ok, f"期望={expected_rate}Hz, 实际≈{actual_rate:.1f}Hz（唯一样本={unique}）"))

    # 清理
    try:
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
    ctrl.terminate()

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("采样率实测汇总", flush=True)
    print("=" * 60, flush=True)
    all_ok = True
    for rate, ok, detail in rate_summaries:
        if ok is None:
            print(f"  [FAIL] EMG_SAMPLE_RATE={rate}：{detail}", flush=True)
            all_ok = False
        elif ok:
            print(f"  [PASS] EMG_SAMPLE_RATE={rate}：{detail}", flush=True)
        else:
            print(f"  [FAIL] EMG_SAMPLE_RATE={rate}：{detail}", flush=True)
            all_ok = False
    print("\n结论: " + ("PASS" if all_ok else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
