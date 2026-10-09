# -*- coding: utf-8 -*-
"""临时脚本：只测 Cerelax Pro 的 IMU 采样率（50/100/200 Hz，各 10 分钟）。

需求：
  - 只测 NTF_IMU，三档 50 / 100 / 200 Hz，每档正式采集 10 分钟。
  - 其他数据流全部关闭（含 NTF_EEG），只保留 NTF_IMU ON。
  - IMU 为 6 通道（acc 3 + gyro 3 聚合）。

注意（按需求执行，但需知风险）：
  - Cerelax 的 IMU 是 EEG 的伴随流（spec co_stream=NTF_EEG），
    EEG 彻底关闭后 IMU 可能录不到任何数据；若如此脚本会明确 FAIL 并提示。

测量口径（沿用 test_measure_sample_rate.py）：
  - 实际采样率 = 唯一样本数 / 精确采集窗口时长（unique = max_idx - min_idx + 1）。
  - 起流后预热 10s 丢弃起流初期不稳定样本，再正式计数。
  - 同时打印设备时间戳口径（absTimeStampInSec）与 host 到达时刻差口径做交叉验证，
    以及每分钟采样率趋势（10 分钟长采，便于看是否随时间漂移）。

用法：
  python temp_imu_only.py
  python temp_imu_only.py 850B   # 指定 Cerelax Pro identity
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import scan_and_match

# ---- 可调参数 ----
RATES = ["50", "100", "200"]       # IMU 三档采样率
MEASURE_SECONDS = 600             # 每档正式采集时长（秒）= 10 分钟
SETTLE_SECONDS = 2.0              # setParam 后静置等待设备异步生效
WARMUP_SECONDS = 10.0             # 起流后预热时长（丢弃起流初期不稳定样本）
READY_TIMEOUT = 15                # 连接后等待 Ready 超时（秒）
FIRST_DATA_TIMEOUT = 10           # 起流后等待首批数据超时（秒）
SAMPLE_RATE_TOLERANCE = 0.001     # 采样率偏差容差 ±0.1%
EXPECTED_CHANNELS = 6             # IMU 期望通道数（acc3 + gyro3）

STREAM = "NTF_IMU"
RATE_KEY = "IMU_SAMPLE_RATE"

# 需要显式关闭的所有数据流（仅保留 NTF_IMU 开启）
ALL_NTF_KEYS = [
    "NTF_EMG", "NTF_GEST", "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
    "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT", "NTF_IMU", "NTF_MAG_ANGLE",
    "NTF_IMPEDANCE", "NTF_EEG", "NTF_ECG", "NTF_BRTH", "NTF_PPG", "NTF_SPO2",
]


class RateCollector:
    """按 DataType 记录样本数、sampleIndex 跨度、首末到达时刻。"""

    def __init__(self):
        self.stats = {}

    def reset(self):
        self.stats = {}

    @property
    def has_data(self):
        return bool(self.stats)

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            try:
                dt = d.getDataType()
            except Exception:
                continue
            s = self.stats.get(dt)
            if s is None:
                s = {
                    "delivered": 0, "channel_count": 0,
                    "first_ts": None, "last_ts": None,
                    "min_idx": None, "max_idx": None,
                    "min_abs_ts": None, "max_abs_ts": None,
                    "timeline": [],
                }
                self.stats[dt] = s
            try:
                n_ch = d.getChannelCount()
                n_smp = d.getSampleCount()
            except Exception:
                n_ch = n_smp = 0
            if n_ch <= 0 or n_smp <= 0:
                continue
            now = time.time()
            if s["first_ts"] is None:
                s["first_ts"] = now
            s["last_ts"] = now
            s["delivered"] += n_ch * n_smp
            if s["channel_count"] == 0:
                s["channel_count"] = n_ch
            batch_min = None
            batch_max = None
            for si in range(n_smp):
                try:
                    idx = d.getSampleIndex(0, si)
                except Exception:
                    continue
                if idx is None:
                    continue
                if batch_min is None or idx < batch_min:
                    batch_min = idx
                if batch_max is None or idx > batch_max:
                    batch_max = idx
                if s["min_idx"] is None or idx < s["min_idx"]:
                    s["min_idx"] = idx
                    try:
                        s["min_abs_ts"] = d.getAbsTimeStampInSec(0, si)
                    except Exception:
                        pass
                if s["max_idx"] is None or idx > s["max_idx"]:
                    s["max_idx"] = idx
                    try:
                        s["max_abs_ts"] = d.getAbsTimeStampInSec(0, si)
                    except Exception:
                        pass
            if batch_min is not None:
                s["timeline"].append((now, batch_min, batch_max))

    def result_for(self, dt):
        s = self.stats.get(dt)
        if not s:
            return 0, 0.0, 0.0, 0, 0
        unique = 0
        if s["min_idx"] is not None and s["max_idx"] is not None:
            unique = s["max_idx"] - s["min_idx"] + 1
        dur_host = 0.0
        if s["first_ts"] is not None and s["last_ts"] is not None:
            dur_host = s["last_ts"] - s["first_ts"]
        dur_abs = 0.0
        if (s["min_abs_ts"] is not None and s["max_abs_ts"] is not None
                and s["max_abs_ts"] > s["min_abs_ts"]):
            dur_abs = s["max_abs_ts"] - s["min_abs_ts"]
        return unique, dur_host, dur_abs, s["delivered"], s["channel_count"]

    def minute_rates(self, dt, t_start, t_end, window=60.0):
        s = self.stats.get(dt)
        if not s or not s["timeline"]:
            return []
        tl = s["timeline"]
        out = []
        i = 0
        n = len(tl)
        k = 0
        while True:
            w_start = t_start + k * window
            if w_start >= t_end:
                break
            w_end = min(w_start + window, t_end)
            seg_min = seg_max = None
            while i < n and tl[i][0] < w_start:
                i += 1
            while i < n and tl[i][0] < w_end:
                ts, mn, mx = tl[i]
                if seg_min is None or mn < seg_min:
                    seg_min = mn
                if seg_max is None or mx > seg_max:
                    seg_max = mx
                i += 1
            if seg_min is None:
                out.append(0.0)
            else:
                out.append(round((seg_max - seg_min + 1) / (w_end - w_start), 3))
            k += 1
        return out


def main():
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None
    ctrl = SensorControllerInstance
    dt_type = getattr(DataType, STREAM, None)

    print("=" * 60, flush=True)
    print("临时脚本：只测 Cerelax Pro IMU 采样率（50/100/200 Hz × 10min）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    if dt_type is None:
        print(f"[FAIL] DataType.{STREAM} 不存在", flush=True)
        ctrl.terminate()
        return

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

    # 关闭所有数据流，仅保留 NTF_IMU ON（按需求：其他数据流全部关闭，含 EEG）
    print("\n[开关] 关闭所有数据流 ...", flush=True)
    for k in ALL_NTF_KEYS:
        if k == STREAM:
            continue
        try:
            sensor.setParam(k, "OFF")
        except Exception as e:
            print(f"  [setParam] {k} OFF 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[开关] 开启 {STREAM} ...", flush=True)
    try:
        sensor.setParam(STREAM, "ON")
    except Exception as e:
        print(f"[setParam] {STREAM} ON 抛异常 {type(e).__name__}: {e}", flush=True)
    print("  提示：IMU 是 EEG 伴随流，EEG 已关闭，若 IMU 无数据属预期（见脚本头注释）", flush=True)

    collector = RateCollector()
    sensor.onDataCallback = collector.on_data

    summaries = []
    for rate in RATES:
        expected = float(rate)
        print("\n" + "=" * 40, flush=True)
        print(f"[档] {STREAM}@{rate}Hz  （正式采集 {MEASURE_SECONDS}s）", flush=True)
        print("=" * 40, flush=True)

        try:
            r = sensor.setParam(RATE_KEY, rate)
        except Exception as e:
            r = f"抛异常 {type(e).__name__}: {e}"
        print(f"[setParam] {RATE_KEY}={rate} -> {r!r}", flush=True)
        if r != "OK":
            summaries.append((f"{STREAM}@{rate}", False, f"setParam {RATE_KEY}={rate} 返回 {r!r}"))
            continue
        try:
            cur = sensor.getParam(RATE_KEY)
            print(f"[getParam] {RATE_KEY} = {cur!r}", flush=True)
        except Exception as e:
            print(f"[getParam] {RATE_KEY} 抛异常 {type(e).__name__}: {e}", flush=True)

        print(f"[等待] 静置 {SETTLE_SECONDS}s 等待采样率生效 ...", flush=True)
        time.sleep(SETTLE_SECONDS)

        collector.reset()
        try:
            sret = sensor.startDataNotification()
        except Exception as e:
            sret = None
            print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)
        print(f"[起流] startDataNotification() -> {sret}", flush=True)
        if sret is not True:
            summaries.append((f"{STREAM}@{rate}", False, "startDataNotification 失败"))
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            continue

        print("[采集] 等待首批数据到达 ...", flush=True)
        t_wait0 = time.time()
        while not collector.has_data and time.time() - t_wait0 < FIRST_DATA_TIMEOUT:
            time.sleep(0.05)

        if not collector.has_data:
            print(f"[采集] {FIRST_DATA_TIMEOUT}s 内未收到任何数据（EEG 已关，IMU 伴随流可能无数据）", flush=True)
            summaries.append((f"{STREAM}@{rate}", False, "起流后无任何数据到达（EEG 关闭，IMU 伴随流无数据）"))
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            continue

        print(f"[采集] 首批已到达，预热 {WARMUP_SECONDS:.0f}s ...", flush=True)
        warmup_end = time.time() + WARMUP_SECONDS
        while time.time() < warmup_end:
            time.sleep(0.05)

        collector.reset()
        t_start = time.time()
        t_end = t_start + MEASURE_SECONDS
        print(f"[采集] 预热结束，正式采集 {MEASURE_SECONDS}s ...", flush=True)
        while time.time() < t_end:
            time.sleep(0.05)

        try:
            sensor.stopDataNotification()
        except Exception as e:
            print(f"[停流] 抛异常 {type(e).__name__}: {e}", flush=True)

        unique, dur_host, dur_abs, delivered, ch = collector.result_for(dt_type)
        if unique <= 0:
            summaries.append((f"{STREAM}@{rate}", False, f"未收到数据（唯一样本={unique}）"))
            continue
        win_dur = t_end - t_start
        actual = unique / win_dur
        dev = abs(actual - expected) / expected if expected > 0 else float("inf")
        ok = dev <= SAMPLE_RATE_TOLERANCE
        ch_note = "" if ch == EXPECTED_CHANNELS else f"  [注意] 通道数={ch}，期望={EXPECTED_CHANNELS}"
        print(f"[结果] {STREAM}@{rate} 期望={expected}Hz, 实际≈{actual:.3f}Hz 偏差={dev:.3%} "
              f"(唯一样本={unique} 投递样本={delivered} 通道={ch} 窗口={win_dur:.1f}s){ch_note}", flush=True)
        if dur_abs > 0:
            print(f"      [口径] 设备时间戳={dur_abs:.3f}s -> {unique / dur_abs:.3f}Hz "
                  f"| host到达时刻差={dur_host:.3f}s -> {unique / dur_host:.3f}Hz", flush=True)
        elif dur_host > 0:
            print(f"      [口径] host到达时刻差={dur_host:.3f}s -> {unique / dur_host:.3f}Hz "
                  f"(设备 absTimeStampInSec 不可用)", flush=True)
        minute_arr = collector.minute_rates(dt_type, t_start, t_end)
        if minute_arr:
            print(f"      [每分钟采样率] {minute_arr}", flush=True)
        summaries.append((f"{STREAM}@{rate}", ok,
                          f"期望={expected}Hz, 实际≈{actual:.3f}Hz 偏差={dev:.3%}（唯一样本={unique}）"))

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    for k in ALL_NTF_KEYS:
        try:
            sensor.setParam(k, "OFF")
        except Exception:
            pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
    ctrl.terminate()

    print("\n" + "=" * 60, flush=True)
    print("IMU 采样率实测汇总", flush=True)
    print("=" * 60, flush=True)
    all_ok = True
    for label, ok, detail in summaries:
        if ok is False:
            print(f"  [FAIL] {label}：{detail}", flush=True)
            all_ok = False
        else:
            print(f"  [PASS] {label}：{detail}", flush=True)
    print("\n结论: " + ("PASS" if all_ok else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
