# -*- coding: utf-8 -*-
"""探测 bug 4662 基线：默认状况（不切换任何滤波开关）下，阻抗是否会出现 0。

设备：OBM5000-8（EB2F），8 通道 EMG。

与 test_4662.py 的区别：
  - 完全不 setParam 任何滤波开关，只连接 -> init -> 起流 -> 观察 60 秒。
  - 读取每个数据包里【所有样本】的阻抗值（而非只看最后一个样本），
    统计每个通道：样本总数 / 非零数 / 零数 / min / max / 首个 0 出现的时刻。

目的：判断「0 阻抗」到底是
  (a) 切换滤波开关导致，还是
  (b) 阻抗本身稀疏采样（部分样本无阻抗值，读出来就是 0），
      即 test_4662.py 只看最后一个样本的口径问题。
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import scan_and_match

OBSERVE_SECONDS = 60.0   # 观察时长（秒）
PRINT_INTERVAL = 1.0     # 打印间隔（秒）
ZERO_EPS = 1e-6          # 判定"阻抗为 0"的阈值


def _dt_name(dt):
    try:
        return dt.name
    except Exception:
        return str(dt)


class AllSampleMonitor:
    """读取每个数据包里所有样本的阻抗值，累计统计每个通道的 0/非零分布。"""

    def __init__(self):
        self.by_ch = {}          # ch -> {n, n_zero, n_nonzero, vmin, vmax, first_zero_ts}
        self.ever_types = set()
        self.samples_by_type = {}   # dt -> 样本总数（用于展示各流数据量，便于排查）
        self.t0 = None

    def _bucket(self, ch):
        b = self.by_ch.get(ch)
        if b is None:
            b = {"n": 0, "n_zero": 0, "n_nonzero": 0,
                 "vmin": None, "vmax": None, "first_zero_ts": None}
            self.by_ch[ch] = b
        return b

    def on_data(self, sensor, data_list):
        if self.t0 is None:
            self.t0 = time.time()
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            try:
                dt = d.getDataType()
                nch = d.getChannelCount()
                nsm = d.getSampleCount()
            except Exception:
                continue
            self.ever_types.add(dt)
            if nch <= 0 or nsm <= 0:
                continue
            self.samples_by_type[dt] = self.samples_by_type.get(dt, 0) + nch * nsm
            # 关键：只有 NTF_IMPEDANCE 流才携带阻抗值，其它流（ACC/GYRO/EEG 等）
            # 的 impedance 字段是默认 0.0（无意义），必须过滤掉，否则会统计出大量"假 0"。
            if dt != DataType.NTF_IMPEDANCE:
                continue
            for ch in range(nch):
                b = self._bucket(ch)
                for si in range(nsm):
                    try:
                        v = d.getImpedance(ch, si)
                    except Exception:
                        continue
                    b["n"] += 1
                    if abs(v) < ZERO_EPS:
                        b["n_zero"] += 1
                        if b["first_zero_ts"] is None:
                            b["first_zero_ts"] = time.time() - self.t0
                    else:
                        b["n_nonzero"] += 1
                        b["vmin"] = v if b["vmin"] is None else min(b["vmin"], v)
                        b["vmax"] = v if b["vmax"] is None else max(b["vmax"], v)


def _fmt_stats(mon, elapsed):
    if not mon.by_ch:
        return "（无阻抗数据）"
    parts = []
    for ch in sorted(mon.by_ch):
        b = mon.by_ch[ch]
        rng = ""
        if b["n_nonzero"] > 0:
            rng = f"[{b['vmin']:.1f}~{b['vmax']:.1f}]"
        parts.append(f"ch{ch}: 零={b['n_zero']}/{b['n']} {rng}")
    return f"{elapsed:4.1f}s  " + " | ".join(parts)


def main():
    ctrl = SensorControllerInstance
    print("=" * 70, flush=True)
    print("bug 4662 基线探测：默认状况下观察阻抗 60 秒（不碰滤波开关）", flush=True)
    print("=" * 70, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    target, _devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[失败] 未匹配到目标设备（EB2F）。", flush=True)
        ctrl.terminate()
        return
    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[失败] requireSensor 返回 None。", flush=True)
        ctrl.terminate()
        return

    print("[连接] connect() ...", flush=True)
    sensor.connect()
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到达 Ready，state={sensor.deviceState}", flush=True)
        ctrl.terminate()
        return

    print(f"[init] init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)

    info = sensor.getDeviceInfo()
    if info is not None:
        try:
            print(f"[DeviceInfo] EmgChannelCount={info.EmgChannelCount} "
                  f"ImpeChannelCount={info.ImpeChannelCount}", flush=True)
        except Exception:
            pass

    monitor = AllSampleMonitor()
    sensor.onDataCallback = monitor.on_data
    for key in ("NTF_EMG", "NTF_IMPEDANCE"):
        try:
            sensor.setParam(key, "ON")
            print(f"[起流] setParam({key}, ON)", flush=True)
        except Exception as e:
            print(f"[起流] setParam({key}, ON) 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        print(f"[起流] startDataNotification() -> {sensor.startDataNotification()}", flush=True)
    except Exception as e:
        print(f"[起流] startDataNotification() 抛异常 {type(e).__name__}: {e}", flush=True)

    # 观察 60 秒
    print(f"\n开始观察 {OBSERVE_SECONDS:.0f} 秒（默认状况，不切换滤波）...", flush=True)
    monitor.t0 = time.time()
    t_start = monitor.t0
    while time.time() - t_start < OBSERVE_SECONDS:
        time.sleep(PRINT_INTERVAL)
        print(_fmt_stats(monitor, time.time() - t_start), flush=True)

    # 收尾
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    for key in ("NTF_EMG", "NTF_IMPEDANCE"):
        try:
            sensor.setParam(key, "OFF")
        except Exception:
            pass
    try:
        sensor.disconnect()
    except Exception:
        pass
    ctrl.terminate()

    # 结论
    print("\n" + "=" * 70, flush=True)
    print("结论", flush=True)
    print("=" * 70, flush=True)
    seen = ", ".join(_dt_name(t) for t in monitor.ever_types) if monitor.ever_types else "无"
    print(f"收到数据类型: {seen}", flush=True)
    if monitor.samples_by_type:
        per_type = ", ".join(f"{_dt_name(t)}={n}" for t, n in sorted(monitor.samples_by_type.items(), key=lambda kv: -kv[1]))
        print(f"各流样本数: {per_type}", flush=True)
    print(f"（仅 NTF_IMPEDANCE 流的阻抗值参与统计，其它流的 impedance 字段为默认 0，已过滤）", flush=True)
    if not monitor.by_ch:
        print("未收到任何阻抗数据，无法判断。请检查电极/连接/设备是否支持阻抗。", flush=True)
        return

    any_zero = any(b["n_zero"] > 0 for b in monitor.by_ch.values())
    total_n = sum(b["n"] for b in monitor.by_ch.values())
    total_zero = sum(b["n_zero"] for b in monitor.by_ch.values())
    print(f"样本统计：共 {total_n} 个阻抗读数，其中 0 值 {total_zero} 个。", flush=True)
    if any_zero:
        print("默认状况下【出现】0 阻抗，说明 0 不是切换滤波引起的。", flush=True)
        for ch in sorted(monitor.by_ch):
            b = monitor.by_ch[ch]
            if b["n_zero"] > 0:
                print(f"  ch{ch}: 首个 0 出现在 {b['first_zero_ts']:.1f}s，"
                      f"零 {b['n_zero']}/{b['n']}", flush=True)
    else:
        print("默认状况下【未出现】0 阻抗（60 秒内所有读数均非零）。", flush=True)
        print("说明 0 更可能与滤波切换相关，可回看 test_4662.py 的 16 组合结果。", flush=True)


if __name__ == "__main__":
    main()
