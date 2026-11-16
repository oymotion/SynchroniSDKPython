# -*- coding: utf-8 -*-
"""验证 PPG 400Hz 下，电池查询频率（powerRefreshInterval）对 PPG 丢包的影响。

目标设备：Cerelax Pro（config.TARGET_IDENTITY 当前默认 851C）。

方法：
  每档完整重连一次（避免跨档状态污染）：
    1) scan + requireSensor + connect 到 Ready
    2) init(packageSampleCount, powerRefreshInterval, timeout)
       —— powerRefreshInterval 即 SDK 内部周期性电池查询（命令 0x08）的间隔
    3) 关闭除 PPG 外的所有流（EEG/IMU/SPO2/IMPEDANCE/...），仅保留 NTF_PPG=ON
    4) setParam("PPG_SAMPLE_RATE", "400")
    5) startDataNotification，采集 5 分钟
    6) 统计 PPG 丢包后 disconnect

4 档 powerRefreshInterval：
  10s（10000）/ 1s（1000）/ 500ms（500）/ 0（不查询）

丢包口径（双重）：
  - gap 丢失样本：按 SensorData.startSampleIndex 连续性累加缺口（即丢失的样本数）
  - SDK lostPackageCount：SDK 自身统计的丢失包计数

前置条件：
  - 电脑蓝牙开启；待测设备 851C 已上电、在范围内
  - 每档 5 分钟，共约 20 分钟 + 连接开销
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))            # .../400 ppg battery/
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))      # .../Automation/
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import scan_and_match

# ---- 复现参数 ----
PPG_SAMPLE_RATE = "400"             # PPG 采样率
PACKAGE_SAMPLE_COUNT = 20           # init(packageSampleCount, ...)
INIT_TIMEOUT_MS = 10000             # init(..., timeoutInMS)
READY_TIMEOUT_S = 15                # 到达 Ready 的等待上限
COLLECT_SECONDS = 300               # 每档采集时长（5 分钟）

# 4 档电池查询间隔（powerRefreshInterval，单位 ms）
TIERS = [
    ("10s",     10000),
    ("1s",      1000),
    ("500ms",   500),
    ("0(不查询)", 0),
]

# 关闭除 PPG 外的所有流（getParam("NTF") 返回的开关 key；Cerelax Pro 无 EMG/ECG 等，
# 对其 setParam OFF 会返回 Error，属预期、不阻断）
STREAMS_OFF = [
    "NTF_ANGLE", "NTF_BRTH", "NTF_ECG", "NTF_EEG", "NTF_EMG",
    "NTF_GEST", "NTF_IMPEDANCE", "NTF_IMU", "NTF_SPO2",
]


def _log_ts(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class PpgMonitor:
    """只统计 PPG 流，按 startSampleIndex 连续性计算丢失样本数。"""

    def __init__(self):
        self.packets = 0
        self.samples = 0
        self.lost_pkg = 0
        self.gap_samples = 0
        self.resets = 0
        self.first_idx = None
        self.next_idx = None
        self.other_types = set()

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            try:
                dt = d.getDataType()
            except Exception:
                continue
            if dt == DataType.NTF_PPG:
                self.packets += 1
                n = d.sampleCount
                self.samples += n
                self.lost_pkg += int(d.lostPackageCount)
                start = int(d.startSampleIndex)
                if self.next_idx is None:
                    self.first_idx = start
                    self.next_idx = start + n
                else:
                    gap = start - self.next_idx
                    if gap > 0:
                        self.gap_samples += gap
                    elif gap < 0:
                        self.resets += 1
                    self.next_idx = start + n
            else:
                self.other_types.add(dt)


def _dt_names(types):
    if not types:
        return "-"
    out = []
    for t in types:
        try:
            out.append(t.name)
        except Exception:
            out.append(str(t))
    return ",".join(sorted(out))


def run_tier(ctrl, interval_ms, label):
    _log_ts(f"===== 档位 {label}：powerRefreshInterval={interval_ms}ms =====")

    target, _ = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        _log_ts(f"[失败] 未匹配到目标设备（identity={config.TARGET_IDENTITY}）")
        return None

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        _log_ts("[失败] requireSensor 返回 None")
        return None

    sensor.autoReconnect = False

    def on_error(s, reason):
        _log_ts(f"[onErrorCallback] {reason!r}")

    sensor.onErrorCallback = on_error

    _log_ts("[profile] connect() ...")
    try:
        sensor.connect()
    except Exception as e:
        _log_ts(f"[profile] connect() 抛异常 {type(e).__name__}: {e}")
        return None

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT_S and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        _log_ts(f"[失败] 未到达 Ready，state={sensor.deviceState}")
        try:
            sensor.disconnect()
        except Exception:
            pass
        return None
    _log_ts("[profile] 到达 Ready")

    # init（powerRefreshInterval 即电池查询间隔）
    _log_ts(f"[profile] init({PACKAGE_SAMPLE_COUNT}, {interval_ms}, {INIT_TIMEOUT_MS}) ...")
    try:
        iret = sensor.init(PACKAGE_SAMPLE_COUNT, interval_ms, INIT_TIMEOUT_MS)
        _log_ts(f"[profile] init() -> {iret}")
    except Exception as e:
        _log_ts(f"[profile] init() 抛异常 {type(e).__name__}: {e}")

    # 关闭除 PPG 外的所有流
    for key in STREAMS_OFF:
        try:
            r = sensor.setParam(key, "OFF")
            _log_ts(f"[setParam] {key}=OFF -> {r!r}")
        except Exception as e:
            _log_ts(f"[setParam] {key}=OFF 抛异常 {type(e).__name__}: {e}")
    try:
        r = sensor.setParam("NTF_PPG", "ON")
        _log_ts(f"[setParam] NTF_PPG=ON -> {r!r}")
    except Exception as e:
        _log_ts(f"[setParam] NTF_PPG=ON 抛异常 {type(e).__name__}: {e}")
    try:
        r = sensor.setParam("PPG_SAMPLE_RATE", PPG_SAMPLE_RATE)
        _log_ts(f"[setParam] PPG_SAMPLE_RATE={PPG_SAMPLE_RATE} -> {r!r}")
    except Exception as e:
        _log_ts(f"[setParam] PPG_SAMPLE_RATE 抛异常 {type(e).__name__}: {e}")

    # 采样率确认：读回 PPG_SAMPLE_RATE，确认设备实际生效的采样率
    try:
        got = sensor.getParam("PPG_SAMPLE_RATE")
        _log_ts(f"[getParam] PPG_SAMPLE_RATE -> {got!r}")
    except Exception as e:
        _log_ts(f"[getParam] PPG_SAMPLE_RATE 抛异常 {type(e).__name__}: {e}")

    monitor = PpgMonitor()
    sensor.onDataCallback = monitor.on_data

    _log_ts("[profile] startDataNotification() ...")
    try:
        sret = sensor.startDataNotification()
        _log_ts(f"[profile] startDataNotification() -> {sret}")
    except Exception as e:
        _log_ts(f"[profile] startDataNotification() 抛异常 {type(e).__name__}: {e}")

    start_ts = time.time()
    _log_ts(f"[采集] 采集 {COLLECT_SECONDS}s ...")
    while time.time() - start_ts < COLLECT_SECONDS:
        time.sleep(10)
        elapsed = time.time() - start_ts
        if sensor.deviceState == DeviceStateEx.Disconnected:
            _log_ts(f"[警告] 采集过程中断开（+{elapsed:.0f}s），提前结束本档")
            break
        _log_ts(f"[进度] {elapsed:.0f}/{COLLECT_SECONDS}s  "
                f"PPG包={monitor.packets} 样本={monitor.samples} "
                f"gap丢失={monitor.gap_samples} SDK丢包={monitor.lost_pkg}")

    elapsed_s = time.time() - start_ts
    try:
        sensor.stopDataNotification()
    except Exception as e:
        _log_ts(f"[profile] stopDataNotification() 抛异常 {type(e).__name__}: {e}")

    metrics = {
        "label": label,
        "interval_ms": interval_ms,
        "packets": monitor.packets,
        "samples": monitor.samples,
        "lost_pkg": monitor.lost_pkg,
        "gap_samples": monitor.gap_samples,
        "resets": monitor.resets,
        "other_types": monitor.other_types,
        "elapsed_s": elapsed_s,
        "effective_rate": (monitor.samples / elapsed_s) if elapsed_s > 0 else 0.0,
    }
    expected = (monitor.next_idx - monitor.first_idx) if (
        monitor.next_idx is not None and monitor.first_idx is not None) else 0
    lost_samples = max(expected - monitor.samples, 0)
    metrics["expected"] = expected
    metrics["lost_samples"] = lost_samples
    metrics["loss_rate_pct"] = (100.0 * lost_samples / expected) if expected > 0 else 0.0

    try:
        sensor.disconnect()
    except Exception:
        pass

    return metrics


def main():
    ctrl = SensorControllerInstance

    print("=" * 78, flush=True)
    print("验证 PPG 400Hz 下，电池查询频率对 PPG 丢包的影响", flush=True)
    print("=" * 78, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"目标设备 identity = {config.TARGET_IDENTITY}", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    results = []
    for label, interval_ms in TIERS:
        m = run_tier(ctrl, interval_ms, label)
        results.append(m)
        time.sleep(2)  # 档间让 BLE 栈冷却

    ctrl.terminate()

    # ---- 汇总表 ----
    print("\n" + "=" * 78, flush=True)
    print("汇总（每档采集 5 分钟，PPG 400Hz，仅开 PPG）", flush=True)
    print("=" * 78, flush=True)
    hdr = (f"{'档位':<10}{'间隔ms':>8}{'PPG包':>8}{'样本数':>10}{'gap丢失':>9}"
           f"{'丢失率%':>9}{'SDK丢包':>9}{'实测Hz':>9}  {'其他流'}")
    print(hdr, flush=True)
    for m in results:
        if m is None:
            print(f"{'<失败>':<10}", flush=True)
            continue
        print(f"{m['label']:<10}{m['interval_ms']:>8}{m['packets']:>8}"
              f"{m['samples']:>10}{m['gap_samples']:>9}{m['loss_rate_pct']:>9.3f}"
              f"{m['lost_pkg']:>9}{m['effective_rate']:>9.1f}  {_dt_names(m['other_types'])}",
              flush=True)
    print("\n说明：", flush=True)
    print("  - gap丢失 = 按 startSampleIndex 连续性统计的丢失样本数（越小越好）", flush=True)
    print("  - 丢失率% = gap丢失 / 期望样本数", flush=True)
    print("  - SDK丢包 = SensorData.lostPackageCount 累加（SDK 自身的丢失包计数）", flush=True)
    print("  - 实测Hz = 收到的样本数 / 实际采集时长（验证是否真在 400Hz 附近）", flush=True)


if __name__ == "__main__":
    main()
