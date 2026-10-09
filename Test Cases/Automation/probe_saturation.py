# -*- coding: utf-8 -*-
"""probe_saturation.py：统计 SensorData 每个样本 saturation 字段的取值分布。

背景：
  saturation 是 Sample 的逐样本 float 字段（offset 28），SDK 文档未给出其物理语义。
  本探针用于「实测」SDK 实际吐出的 saturation 值长什么样，据此反推正确语义，
  排查「saturation 不正确」这类空描述 bug。

对每个 DataType 输出：
  1) saturation 取值分布（min/max/mean + 各不同取值及计数，浮点按 ROUND 位合并噪声）
  2) 非零占比（判断它是不是「0=正常 / 非0=异常」的二值/稀疏信号）
  3) rawData 实测范围 [min, max]（作为 ADC 量程上下沿的参考）
  4) sat≠0 的样本示例（raw, sat），以及 raw 处于本批 min/max 边沿的样本示例（raw, sat）
     —— 用于交叉验证「saturation 非零是否与 rawData 撞量程（削顶）相关」。

用法：
    python probe_saturation.py                 # 用下方 PROBE_STREAMS 默认流
    python probe_saturation.py NTF_EEG NTF_PPG # 指定要探的流（覆盖默认）

设备与流（按需改 PROBE_STREAMS 或传参）：
  - gForceUltra : NTF_EMG / NTF_GEST / NTF_IMU / NTF_IMPEDANCE
  - Cerelax Pro : NTF_EEG / NTF_IMU / NTF_IMPEDANCE / NTF_PPG
  - OB6000C     : NTF_EEG / NTF_ECG / NTF_IMU 等

前置条件：目标设备上电、在范围内（identity 见 config.TARGET_IDENTITY）。
"""

import os
import sys
import time
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import numpy as np
except ImportError:
    np = None

from sensor import *
import config
import common
from common import scan_and_match

# ---- 可调参数 ----
PROBE_STREAMS = ["NTF_EMG", "NTF_GEST", "NTF_IMU", "NTF_IMPEDANCE"]
COLLECT_SECONDS = 3          # 每路流采集时长（秒）
ROUND = 4                    # saturation 浮点合并位数（合并浮点噪声）
TOP_N = 20                   # 分布里展示的出现次数最多的 N 个取值
MAX_EXAMPLES = 5             # 每类示例最多保留条数

# 清理时统一 OFF 的 NTF key（与 probe_data_types.py 一致）
ALL_NTF_KEYS = [
    "NTF_GEST", "NTF_EMG", "NTF_EEG", "NTF_ECG", "NTF_IMU",
    "NTF_BRTH", "NTF_IMPEDANCE", "NTF_MAG_ANGLE",
    "NTF_PPG", "NTF_SPO2",
    "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT", "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
]


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


class SaturationCollector:
    """按 DataType 累积 saturation 取值分布 + rawData 范围 + 交叉示例。"""

    def __init__(self):
        self.stats = {}

    def _entry(self, dt):
        return self.stats.setdefault(dt, {
            "samples": 0,
            "sat_min": None,
            "sat_max": None,
            "sat_sum": 0.0,
            "sat_counter": Counter(),
            "sat_nonzero": 0,
            "raw_min": None,
            "raw_max": None,
            "sat_nonzero_examples": [],
            "raw_edge_examples": [],
        })

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            try:
                arr = d.as_numpy()
            except Exception:
                continue
            if arr is None or arr.size == 0:
                continue
            sat = arr["saturation"].ravel()
            raw = arr["rawData"].ravel()
            e = self._entry(dt)
            n = int(sat.size)
            if n == 0:
                continue
            e["samples"] += n

            # saturation min/max/mean
            s_min = float(sat.min())
            s_max = float(sat.max())
            e["sat_min"] = s_min if e["sat_min"] is None else min(e["sat_min"], s_min)
            e["sat_max"] = s_max if e["sat_max"] is None else max(e["sat_max"], s_max)
            e["sat_sum"] += float(sat.sum())

            # rawData 范围（int32）
            r_min = int(raw.min())
            r_max = int(raw.max())
            e["raw_min"] = r_min if e["raw_min"] is None else min(e["raw_min"], r_min)
            e["raw_max"] = r_max if e["raw_max"] is None else max(e["raw_max"], r_max)

            # saturation 分布（ROUND 位合并）
            rounded = np.round(sat, ROUND)
            for v, c in zip(*np.unique(rounded, return_counts=True)):
                e["sat_counter"][float(v)] += int(c)

            e["sat_nonzero"] += int(np.count_nonzero(sat))

            # 交叉示例
            for v_raw, v_sat in zip(raw, sat):
                if float(v_sat) != 0.0 and len(e["sat_nonzero_examples"]) < MAX_EXAMPLES:
                    e["sat_nonzero_examples"].append((int(v_raw), float(v_sat)))
                if (int(v_raw) == r_min or int(v_raw) == r_max) and len(e["raw_edge_examples"]) < MAX_EXAMPLES:
                    e["raw_edge_examples"].append((int(v_raw), float(v_sat)))
                if (len(e["sat_nonzero_examples"]) >= MAX_EXAMPLES
                        and len(e["raw_edge_examples"]) >= MAX_EXAMPLES):
                    break

    def clear(self):
        self.stats.clear()


def _dump(collector, label):
    print(f"\n--- [{label}] saturation 取值分布 ---", flush=True)
    if not collector.stats:
        print("  未收到任何数据", flush=True)
        return
    for dt, e in sorted(collector.stats.items(), key=lambda kv: kv[0]):
        n = e["samples"]
        mean = (e["sat_sum"] / n) if n else 0.0
        nz_ratio = (e["sat_nonzero"] / n * 100.0) if n else 0.0
        print(f"\n  DataType={_dt_name(dt)}  样本数={n}", flush=True)
        print(f"    saturation: min={e['sat_min']!r} max={e['sat_max']!r} "
              f"mean={mean:.6g}  非零={e['sat_nonzero']} ({nz_ratio:.4f}%)", flush=True)
        print(f"    rawData 实测范围: [{e['raw_min']}, {e['raw_max']}]", flush=True)
        if e["sat_counter"]:
            top = e["sat_counter"].most_common(TOP_N)
            n_distinct = len(e["sat_counter"])
            print(f"    saturation 取值分布（top {len(top)} / 共 {n_distinct} 个不同值）:", flush=True)
            for v, c in top:
                print(f"        {v!r}: {c}", flush=True)
        if e["sat_nonzero_examples"]:
            print(f"    sat≠0 示例 (raw, sat): {e['sat_nonzero_examples']}", flush=True)
        else:
            print("    sat≠0 示例: 无（全为 0）", flush=True)
        if e["raw_edge_examples"]:
            print(f"    raw 处于本批 min/max 边沿示例 (raw, sat): {e['raw_edge_examples']}", flush=True)


def _probe(sensor, collector, keys, collect_sec, label):
    print(f"\n{'=' * 50}", flush=True)
    print(f"[探针] {label}", flush=True)
    print(f"[探针] 开启 {keys}，采集 {collect_sec}s", flush=True)
    print("=" * 50, flush=True)

    # 清理：全部 OFF
    for k in ALL_NTF_KEYS:
        try:
            sensor.setParam(k, "OFF")
        except Exception as e:
            print(f"  [清理] {k} OFF 抛异常 {type(e).__name__}: {e}", flush=True)

    # 开目标 key
    for k in keys:
        try:
            r = sensor.setParam(k, "ON")
            print(f"  [setParam] {k} ON -> {r!r}", flush=True)
        except Exception as e:
            print(f"  [setParam] {k} ON 抛异常 {type(e).__name__}: {e}", flush=True)

    collector.clear()
    try:
        sr = sensor.startDataNotification()
        print(f"  [起流] startDataNotification() -> {sr!r}", flush=True)
    except Exception as e:
        print(f"  [起流] 抛异常 {type(e).__name__}: {e}", flush=True)
        return

    time.sleep(collect_sec)

    try:
        sensor.stopDataNotification()
    except Exception:
        pass

    for k in keys:
        try:
            sensor.setParam(k, "OFF")
        except Exception:
            pass

    _dump(collector, label)


def main():
    ctrl = SensorControllerInstance

    # 支持命令行覆盖流列表
    streams = sys.argv[1:] or PROBE_STREAMS

    print("=" * 60, flush=True)
    print("probe_saturation：saturation 取值分布诊断", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"探针流列表 = {streams}", flush=True)

    if np is None:
        print("[失败] 需要 numpy（pip install numpy），本探针依赖 as_numpy() 加速。", flush=True)
        ctrl.terminate()
        return

    is_enable = ctrl.isEnable
    print(f"[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[失败] 未匹配到 config 中启用的设备", flush=True)
        ctrl.terminate()
        return
    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    print("[连接] SensorProfile.connect() ...", flush=True)
    sensor.connect()
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到达 Ready，state={sensor.deviceState}", flush=True)
        ctrl.terminate()
        return
    print(f"[连接] 到达 Ready，state={sensor.deviceState}", flush=True)

    print(f"[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)

    collector = SaturationCollector()
    sensor.onDataCallback = collector.on_data

    for k in streams:
        _probe(sensor, collector, [k], COLLECT_SECONDS, k)

    # 清理
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception:
        pass
    ctrl.terminate()

    print("\n" + "=" * 60, flush=True)
    print("诊断完成。判断口径：", flush=True)
    print("  - 若 saturation 绝大多数为 0、仅在 raw 撞量程时非 0 → 它是 ADC 削顶标志。", flush=True)
    print("  - 若始终为 0 → 可能 SDK 未填/恒错，或该流本就不产生 saturation。", flush=True)
    print("  - 若非 0 但 raw 未撞量程（raw 不在 min/max 边沿）→ 疑似『saturation 不正确』候选反例。", flush=True)
    print("=" * 60, flush=True)


if __name__ == "__main__":
    main()
