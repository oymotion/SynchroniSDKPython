# -*- coding: utf-8 -*-
"""验证 bug 4662：切换滤波开关导致阻抗值变为 0。

设备：OBM5000-8（EB2F），8 通道 EMG。
现象：连接设备后反复切换 4 个滤波开关（50Hz 陷波 / 60Hz 陷波 / HPF / LPF），
      某一组合下阻抗值（样本 impedance 字段，getImpedance(ci,si)）变为 0。

方法：
  1) 连接 + init 设备
  2) 起流（NTF_EMG + NTF_IMPEDANCE），记录每个通道的最新阻抗值
  3) 穷举 2^4=16 种滤波开关组合（all OFF 排在最前，作为基线）
  4) 每种组合：setParam 应用该组合 -> 连续 5 秒输出各通道阻抗值 -> 检测是否出现 0
  5) 输出每种组合的阻抗读数，并标注出现 0 的组合

说明：
  - 4 个滤波开关是 SDK 固定参数名（值 ON/OFF），不存在设备差异。
  - 阻抗值读取口径：样本的 impedance 字段（getImpedance），单位欧姆(ohm)，
    与 demo 中显示的阻抗（再除以 1000 得 KOhm）一致。
  - 阻抗 co-stream 依赖 EMG：必须同时起 NTF_EMG，否则 NTF_IMPEDANCE 可能不推数据。
  - 电极未贴合时阻抗值会很高（不会是 0），因此出现 0 即为异常（bug 复现）。
"""

import itertools
import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import scan_and_match

FILTER_KEYS = ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]

OBSERVE_SECONDS = 5.0   # 每种组合设置后连续输出阻抗值的时长（秒）
PRINT_INTERVAL = 0.5    # 观察期内打印间隔（秒）
ZERO_EPS = 1e-6         # 判定"阻抗为 0"的阈值


def _dt_name(dt):
    try:
        return dt.name
    except Exception:
        return str(dt)


class ImpedanceMonitor:
    """记录最新收到的各通道阻抗值（impedance 字段）。

    latest / types：随每次 clear() 重置（用于单种组合的观察窗口）。
    ever_latest / ever_types：全程累计，不重置（用于最终有无数据的判定）。
    """

    def __init__(self):
        self.latest = {}
        self.types = set()
        self.ever_latest = {}
        self.ever_types = set()

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        for d in items:
            try:
                dt = d.getDataType()
                nch = d.getChannelCount()
                nsm = d.getSampleCount()
            except Exception:
                continue
            if nch <= 0 or nsm <= 0:
                continue
            self.types.add(dt)
            self.ever_types.add(dt)
            # 关键：只有 NTF_IMPEDANCE 流携带阻抗值；其它流 impedance 默认 0，必须过滤，
            # 否则会把 ACC/GYRO/EEG 等流的"假 0"误判成阻抗为 0。
            if dt != DataType.NTF_IMPEDANCE:
                continue
            for ch in range(nch):
                try:
                    v = d.getImpedance(ch, nsm - 1)
                except Exception:
                    continue
                self.latest[ch] = v
                self.ever_latest[ch] = v

    def clear(self):
        self.latest = {}
        self.types = set()


def _apply_filters(sensor, combo):
    for key, val in zip(FILTER_KEYS, combo):
        try:
            sensor.setParam(key, val)
        except Exception as e:
            print(f"    [setParam] {key}={val} 抛异常 {type(e).__name__}: {e}", flush=True)


def _fmt_impedance(d):
    if not d:
        return "（无阻抗数据）"
    parts = [f"ch{ch}={v:.1f}" for ch, v in sorted(d.items())]
    return " ".join(parts)


def _has_zero(d):
    return any(abs(v) < ZERO_EPS for v in d.values())


def main():
    ctrl = SensorControllerInstance
    print("=" * 70, flush=True)
    print("bug 4662 验证：切换滤波开关导致阻抗值为 0", flush=True)
    print("=" * 70, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描 + 连接
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

    # 起流：EMG + IMPEDANCE（阻抗 co-stream 依赖 EMG）
    monitor = ImpedanceMonitor()
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

    # 等第一波数据
    for _ in range(30):
        if monitor.latest:
            break
        time.sleep(0.1)
    seen = ", ".join(_dt_name(t) for t in monitor.ever_types) if monitor.ever_types else "无"
    print(f"[起流] 收到数据类型: {seen}", flush=True)
    if not monitor.latest:
        print("[警告] 未收到阻抗数据。请确认电极已贴好、设备支持阻抗检测。", flush=True)

    # 穷举 16 种组合（all OFF 排在最前，作为基线）
    zero_combos = []
    print("\n穷举 16 种滤波开关组合（每种连续观察 5 秒）：", flush=True)
    for combo in itertools.product(["OFF", "ON"], repeat=len(FILTER_KEYS)):
        desc = ", ".join(f"{k}={v}" for k, v in zip(FILTER_KEYS, combo))
        print(f"\n[组合] {desc}", flush=True)
        _apply_filters(sensor, list(combo))
        monitor.clear()
        t0 = time.time()
        combo_zero = False
        while time.time() - t0 < OBSERVE_SECONDS:
            time.sleep(PRINT_INTERVAL)
            print(f"    {time.time() - t0:4.1f}s  {_fmt_impedance(monitor.latest)}", flush=True)
            if _has_zero(monitor.latest):
                combo_zero = True
        if combo_zero:
            zero_combos.append(desc)
            print("    <== 该组合下阻抗出现 0", flush=True)

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
    if not monitor.ever_latest:
        print("未收到任何阻抗数据，无法验证（请检查设备/电极/连接）。", flush=True)
    elif zero_combos:
        print(f"复现 bug：以下组合下阻抗值出现 0（共 {len(zero_combos)} 种）：", flush=True)
        for d in zero_combos:
            print(f"  - {d}", flush=True)
    else:
        print("本次未复现（16 种组合下阻抗均未出现 0）。可考虑重复多轮 / 随机顺序再试。", flush=True)


if __name__ == "__main__":
    main()
