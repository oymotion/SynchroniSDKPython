# -*- coding: utf-8 -*-
"""probe_data_types.py：诊断 OB6000C 各起流模式实际产出的 DataType 与样本结构。

用途（定位 DATA-FUNC-005 里 IMPEDANCE / ACC / GYRO / EULER / QUAT 收不到数据）：
  1) NTF_IMPEDANCE 单独起流：dump 实际收到的 DataType 全集（确认阻抗是独立流还是内嵌）
  2) NTF_IMU 起流：dump DataType 全集 + 通道数（确认 IMU 聚合流）
  3) NTF_GFORCE_ACC/GYRO/EULER/QUAT 四路单独起流（复现原脚本逻辑）：dump DataType 全集
  4) NTF_EEG 起流：dump DataType 全集 + 检查 EEG 样本 impedance 字段是否非零

每个探针前先把所有 NTF_* 置 OFF 清理状态，避免串扰。

用法：
    python probe_data_types.py

设备信息见 config.py（TARGET_IDENTITY 当前指向 OB6000C）。
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor import *
import config
import common
from common import scan_and_match

# 所有需要清理的 NTF key（来自 README「Data stream toggles」）
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


class ProbeCollector:
    """按 DataType 统计批次/样本数，并可采集样本的 impedance 字段样本值。"""

    def __init__(self):
        self.by_type = {}      # {DataType: {'batches': n, 'samples': m}}
        self.impedance_values = []  # 采集到的 impedance 字段值（前若干）
        self.sample_fields = {}     # 第一次出现的样本字段名 dump

    def on_data(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = d.getDataType()
            entry = self.by_type.setdefault(dt, {'batches': 0, 'samples': 0})
            entry['batches'] += 1
            cs = getattr(d, 'channelSamples', None)
            if cs:
                try:
                    entry['samples'] += sum(len(ch) for ch in cs)
                except TypeError:
                    entry['samples'] += len(cs)

            # dump 样本字段名（仅一次）
            if not self.sample_fields and cs:
                first_ch = cs[0] if cs else None
                if first_ch and len(first_ch) > 0:
                    s0 = first_ch[0]
                    for f in ['data', 'rawData', 'impedance', 'saturation',
                              'sampleIndex', 'channelIndex', 'absTimeStampInSec', 'isLost']:
                        try:
                            self.sample_fields[f] = getattr(s0, f, '<无>')
                        except Exception as e:
                            self.sample_fields[f] = f"<{type(e).__name__}>"

            # 采集 impedance 字段值（前 10 个非零，用于判断是否内嵌阻抗）
            if cs and len(self.impedance_values) < 10:
                for ch in cs:
                    for s in ch:
                        try:
                            imp = s.impedance
                        except Exception:
                            continue
                        if imp not in (None, 0, 0.0):
                            self.impedance_values.append(imp)
                            if len(self.impedance_values) >= 10:
                                break

    def clear(self):
        self.by_type.clear()
        self.impedance_values.clear()
        self.sample_fields.clear()

    def summary(self):
        return {_dt_name(k): v['batches'] for k, v in self.by_type.items()}


def _dump_collector(collector, label):
    print(f"\n--- [{label}] 采集结果 ---", flush=True)
    if not collector.by_type:
        print("  未收到任何数据", flush=True)
    else:
        for dt, entry in collector.by_type.items():
            print(f"  DataType={_dt_name(dt)}  批次={entry['batches']}  样本数={entry['samples']}", flush=True)
    if collector.sample_fields:
        print(f"  样本字段样例: {collector.sample_fields}", flush=True)
    if collector.impedance_values:
        print(f"  impedance 字段非零值样例（前10）: {collector.impedance_values}", flush=True)
    else:
        print("  impedance 字段非零值: 无（均为 0/None 或未采集到）", flush=True)


def _probe(sensor, collector, keys, collect_sec, label):
    """清理状态 → 开指定 keys → 起流采集 → 停流关 key → dump。"""
    print(f"\n{'=' * 50}", flush=True)
    print(f"[探针] {label}", flush=True)
    print(f"[探针] 开启 {keys}，采集 {collect_sec}s", flush=True)
    print('=' * 50, flush=True)

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

    # 起流采集
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

    _dump_collector(collector, label)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("probe_data_types：OB6000C 各起流模式 DataType 诊断", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

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

    collector = ProbeCollector()
    sensor.onDataCallback = collector.on_data

    # 探针 1：NTF_IMPEDANCE 单独起流（1Hz，采集 25s 凑满一批 packageSampleCount=20）
    _probe(sensor, collector, ["NTF_IMPEDANCE"], 25, "IMPEDANCE 单独起流")

    # 探针 2：NTF_IMU 聚合流
    _probe(sensor, collector, ["NTF_IMU"], 5, "IMU 聚合流")

    # 探针 3：四路 NTF_GFORCE_* 单独起流（复现原脚本逻辑）
    _probe(sensor, collector,
           ["NTF_GFORCE_ACC", "NTF_GFORCE_GYRO", "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT"],
           5, "四路 NTF_GFORCE_* 单独起流")

    # 探针 4：NTF_EEG 起流，检查 impedance 字段是否内嵌
    _probe(sensor, collector, ["NTF_EEG"], 5, "EEG 起流 + impedance 字段检查")

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
    print("诊断完成。请根据上方 DataType 输出判断：", flush=True)
    print("  - IMPEDANCE 是否为独立流（出现 NTF_IMPEDANCE/NTF_IMPEDANCE_EXT）", flush=True)
    print("  - ACC/GYRO/EULER/QUAT 是否以 NTF_IMU 聚合流形式出现", flush=True)
    print("  - EEG 样本 impedance 字段是否有非零值（判断阻抗内嵌）", flush=True)
    print("=" * 60, flush=True)


if __name__ == "__main__":
    main()
