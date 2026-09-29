# -*- coding: utf-8 -*-
"""临时探测：Cerelax Pro 的 PPG 是否伴随 EEG（co_stream），还是独立流。

目的：确定 spec["sample_rates"]["PPG_SAMPLE_RATE"]["co_stream"] 该填什么。
  - 若 PPG 仅在 EEG 起流时才有数据  -> co_stream = "NTF_EEG"
  - 若 PPG 可单独起流（不依赖 EEG）-> co_stream = None

两个场景：
  A) 只开 NTF_PPG（关 EEG/IMU/阻抗）起流，看 PPG 是否有数据
  B) 只开 NTF_EEG（关 PPG/IMU/阻抗）起流，看 PPG 是否伴随出现

用法：
  python probe_ppg_costream.py
  python probe_ppg_costream.py 851C

前置条件：待测 Cerelax Pro（851C）上电、在范围内。
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

COLLECT_SECONDS = 6        # 每场景采集时长（秒），仅探测有无数据，无需长测
SETTLE_SECONDS = 1.0       # setParam 后静置
READY_TIMEOUT = 15
FIRST_DATA_TIMEOUT = 10

STREAMS = ["NTF_EEG", "NTF_IMU", "NTF_IMPEDANCE", "NTF_PPG"]

# DataType int -> 名称 反向映射
_DT_NAMES = {}
for _n in dir(DataType):
    if _n.startswith("NTF_"):
        try:
            _DT_NAMES[int(getattr(DataType, _n))] = _n
        except Exception:
            pass


def dt_name(v):
    return _DT_NAMES.get(v, str(v))


class StreamProbe:
    """记录起流窗口内出现的 DataType 及样本数。"""

    def __init__(self):
        self.samples = {}   # datatype -> 样本数

    def reset(self):
        self.samples = {}

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            try:
                n_smp = d.getSampleCount()
            except Exception:
                n_smp = 0
            if n_smp <= 0:
                continue
            self.samples[dt] = self.samples.get(dt, 0) + n_smp


def _set_streams(sensor, on_streams, off_streams):
    for s in off_streams:
        try:
            sensor.setParam(s, "OFF")
        except Exception as e:
            print(f"  [setParam] {s} OFF 抛异常 {type(e).__name__}: {e}", flush=True)
    for s in on_streams:
        try:
            r = sensor.setParam(s, "ON")
            print(f"  [setParam] {s} ON -> {r!r}", flush=True)
        except Exception as e:
            print(f"  [setParam] {s} ON 抛异常 {type(e).__name__}: {e}", flush=True)


def run_scenario(sensor, probe, on_streams, off_streams, label):
    print("\n" + "=" * 50, flush=True)
    print(f"[场景] {label}", flush=True)
    _set_streams(sensor, on_streams, off_streams)
    time.sleep(SETTLE_SECONDS)

    probe.reset()
    try:
        sret = sensor.startDataNotification()
    except Exception as e:
        sret = None
        print(f"[起流] 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[起流] startDataNotification() -> {sret}", flush=True)
    if sret is not True:
        try:
            sensor.stopDataNotification()
        except Exception:
            pass
        return set(probe.samples.keys())

    time.sleep(COLLECT_SECONDS)

    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] 抛异常 {type(e).__name__}: {e}", flush=True)

    seen = set(probe.samples.keys())
    if seen:
        for dt in sorted(seen, key=str):
            print(f"  收到 {dt_name(dt)}：{probe.samples[dt]} 样本", flush=True)
    else:
        print("  （未收到任何数据）", flush=True)
    return seen


def main():
    target_identity = sys.argv[1].strip().upper() if len(sys.argv) > 1 else None
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("Cerelax Pro：探测 PPG 是否伴随 EEG（co_stream）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

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

    probe = StreamProbe()
    sensor.onDataCallback = probe.on_data

    # 场景 A：只开 PPG
    seen_a = run_scenario(
        sensor, probe,
        on_streams=["NTF_PPG"],
        off_streams=["NTF_EEG", "NTF_IMU", "NTF_IMPEDANCE"],
        label="A) 只开 NTF_PPG（关 EEG/IMU/阻抗）",
    )

    # 场景 B：只开 EEG
    seen_b = run_scenario(
        sensor, probe,
        on_streams=["NTF_EEG"],
        off_streams=["NTF_PPG", "NTF_IMU", "NTF_IMPEDANCE"],
        label="B) 只开 NTF_EEG（关 PPG/IMU/阻抗）",
    )

    ppg_a = DataType.NTF_PPG in seen_a
    ppg_b = DataType.NTF_PPG in seen_b

    # 清理
    for s in STREAMS:
        try:
            sensor.setParam(s, "OFF")
        except Exception:
            pass
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
    ctrl.terminate()

    print("\n" + "=" * 60, flush=True)
    print("结论", flush=True)
    print("=" * 60, flush=True)
    print(f"  A) 只开 PPG 时，PPG 是否有数据：{'是' if ppg_a else '否'}", flush=True)
    print(f"  B) 只开 EEG 时，PPG 是否伴随出现：{'是' if ppg_b else '否'}", flush=True)

    if ppg_a:
        print("  => PPG 可独立起流，co_stream 应为 None", flush=True)
    elif ppg_b:
        print("  => PPG 需伴随 EEG，co_stream 应为 'NTF_EEG'", flush=True)
    else:
        print("  => 两个场景都无 PPG 数据，PPG 可能另有前置条件，需进一步排查", flush=True)


if __name__ == "__main__":
    main()
