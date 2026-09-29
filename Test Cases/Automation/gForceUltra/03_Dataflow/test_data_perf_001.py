# -*- coding: utf-8 -*-
"""DATA-PERF-001：实测采样率与标称偏差 ≤ 容差。

对应用例：03_数据流.md -> DATA-PERF-001
可自动化：auto（设备上电、在范围内为运行前置，测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) setParam("NTF_EMG", "ON") 起 EMG 流（gForceUltra 起流会伴随 IMU 等其它流）
  3) startDataNotification 后采集窗口内，按 DataType 分别统计样本
  4) 对每一路流：实际采样率与各自 getSampleRate() 标称比较，偏差 ≤ 容差（默认 ±0.1%）

说明：
  实测采样率用"固定窗口计时"计算：从首批数据到达起，精确采集 PERF_COLLECT_SECONDS
  秒，用固定墙钟时长做分母（而非首末批到达时间），彻底排除起流前延迟和停流空转：
      实测采样率 = 该流去重后唯一样本数（sampleIndex 首末 span）/ PERF_COLLECT_SECONDS
  不同 DataType（EMG 500/1000Hz / IMU 50Hz / IMPEDANCE 等）采样率不同，且各流
  sampleIndex 各自独立从 0 递增，故必须按 getDataType() 隔离后逐流比较，不能混在
  一起统计（混流会把不同采样率的样本加总，得出虚高、也谈不上"重复"）。
  主模态（EMG）流的标称采样率取自该流首批 SensorData.getSampleRate()（动态，500/1000）。
  阻抗（电极接触检测）流固定 1Hz：SDK 的 getSampleRate() 对阻抗流同样返回主模态采样率
  （如 500），不反映阻抗真实速率（实测 1Hz），故阻抗流标称单独用 IMPEDANCE_NOMINAL_HZ。
  注：早期 SDK 曾存在 init 时把 EMG 采样率静默改为 500 的 bug（现已修复），
  当前 init 不再改采样率，上电默认 1000；本用例仍以 getSampleRate() 返回的
  标称为准逐流比对，不依赖固定默认值。
  采样率统计与是否佩戴无关（佩戴只影响信号内容，不影响采样速率）。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：gForceUltra 上电、在范围内
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

SAMPLE_RATE_TOLERANCE = 0.001  # 采样率容差 ±0.1%
MIN_SAMPLES_FOR_RATE = 100    # 计算采样率所需最小样本数（保证统计精度）
PERF_COLLECT_SECONDS = 30     # 固定窗口采集时长（秒），用于实测采样率统计
IMPEDANCE_NOMINAL_HZ = 1.0    # 阻抗流固定采样率（SDK 不单独上报，实测 1Hz）


def _dt_name(dt):
    try:
        if isinstance(dt, DataType):
            return dt.name
        return DataType(dt).name
    except Exception:
        return str(dt)


def _nominal_for(dt, sensor_data):
    """返回某 DataType 的标称采样率（Hz）。

    主模态（EMG）采样率动态，取 SensorData.getSampleRate()；
    阻抗流固定 1Hz：getSampleRate() 对阻抗流也返回主模态采样率（如 500），
    不反映阻抗真实速率，故阻抗流标称用 IMPEDANCE_NOMINAL_HZ。
    """
    if dt == DataType.NTF_IMPEDANCE:
        return IMPEDANCE_NOMINAL_HZ
    try:
        return float(sensor_data.getSampleRate())
    except Exception:
        return None


class RateCollector:
    """按 DataType 分别统计样本，逐流计算各自的实际采样率。"""

    def __init__(self):
        self.first_ts = None     # 首批到达时间（任意流，作为固定窗口起点）
        self.last_ts = None      # 末批到达时间
        self.batches = 0
        # {data_type: {"delivered", "channels", "min_idx", "max_idx", "nominal_rate"}}
        self.by_type = {}

    def on_data(self, sensor, data):
        now = time.time()
        if self.first_ts is None:
            self.first_ts = now
        self.last_ts = now
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.batches += 1
            try:
                dt = d.getDataType()
            except Exception:
                continue
            try:
                n_ch = d.getChannelCount()
                n_smp = d.getSampleCount()
            except Exception:
                n_ch = 0
                n_smp = 0
            if n_ch <= 0 or n_smp <= 0:
                continue
            stat = self.by_type.get(dt)
            if stat is None:
                stat = {
                    "delivered": 0,
                    "channels": n_ch,
                    "min_idx": None,
                    "max_idx": None,
                    "nominal_rate": None,
                }
                self.by_type[dt] = stat
            stat["delivered"] += n_smp
            if stat["nominal_rate"] is None:
                stat["nominal_rate"] = _nominal_for(dt, d)
            # 通道0 sampleIndex 记录该流的首末（去重后唯一样本数）
            for si in range(n_smp):
                try:
                    idx = d.getSampleIndex(0, si)
                except Exception:
                    continue
                if idx is None:
                    continue
                if stat["min_idx"] is None or idx < stat["min_idx"]:
                    stat["min_idx"] = idx
                if stat["max_idx"] is None or idx > stat["max_idx"]:
                    stat["max_idx"] = idx


def _unique(stat):
    if stat["min_idx"] is None or stat["max_idx"] is None:
        return None
    return stat["max_idx"] - stat["min_idx"] + 1


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-PERF-001 实测采样率与标称偏差 <= 容差", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：gForceUltra 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 gForceUltra 已【开机】且在范围内，"
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

    # 起 EMG 流
    print("\n[起流] SensorProfile.setParam('NTF_EMG', 'ON') ...", flush=True)
    try:
        p_ret = sensor.setParam("NTF_EMG", "ON")
        p_txt = f"返回 {p_ret!r}"
    except Exception as e:
        p_ret = None
        p_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] setParam('NTF_EMG', 'ON') -> {p_txt}", flush=True)

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
        sensor.setParam("NTF_EMG", "OFF")
    except Exception:
        pass

    # 按 DataType 逐流统计实测采样率，与各自标称采样率对比
    print(f"\n[统计] 批次数={collector.batches}", flush=True)
    print(f"[统计] 收到数据类型: {[_dt_name(dt) for dt in collector.by_type]}", flush=True)
    print(f"[统计] 固定窗口时长={window_duration if window_duration is not None else 'N/A'}s", flush=True)

    record(results, "收到数据（首批已到达）", collector.first_ts is not None,
           "首批数据已到达", "未收到数据" if collector.first_ts is None else "已收到数据")

    if window_duration is None or not collector.by_type:
        record(results, "实测采样率偏差 <= 容差（逐流去重后）", False,
               f"各数据类型 |实测-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.1%}",
               "未收到任何数据，无法计算实测采样率")
    else:
        for dt, stat in sorted(collector.by_type.items(), key=lambda kv: _dt_name(kv[0])):
            name = _dt_name(dt)
            nominal = stat["nominal_rate"]
            delivered = stat["delivered"]
            unique = _unique(stat)
            measured_rate = (unique / window_duration) if unique else (delivered / window_duration)
            delivered_rate = delivered / window_duration

            print(f"\n[{name}] 标称={nominal}Hz 通道={stat['channels']} "
                  f"唯一样本={unique} 投递样本={delivered}", flush=True)
            print(f"[{name}] 实测(去重)={measured_rate:.1f}Hz 实测(投递)={delivered_rate:.1f}Hz", flush=True)

            if unique is not None and unique < MIN_SAMPLES_FOR_RATE:
                print(f"[{name}] 唯一样本 < {MIN_SAMPLES_FOR_RATE}，采样率统计精度可能不足", flush=True)

            if nominal is None or nominal <= 0:
                record(results, f"{name} 实测采样率偏差 <= 容差", False,
                       f"|实测-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.1%}",
                       f"标称采样率非法（{nominal}），无法比较")
                continue

            deviation = abs(measured_rate - nominal) / nominal
            record(results, f"{name} 实测采样率偏差 <= 容差（去重后）", deviation <= SAMPLE_RATE_TOLERANCE,
                   f"|实测(去重)-标称|/标称 <= {SAMPLE_RATE_TOLERANCE:.1%}",
                   f"实测(去重)={measured_rate:.1f}Hz 实测(投递)={delivered_rate:.1f}Hz 标称={nominal:.1f}Hz 偏差={deviation:.2%}")

            # 交叉验证（同一路流内）：投递样本数 vs 唯一样本数，有差异才可能是重复投递
            if unique:
                diff_ratio = abs(delivered - unique) / unique
                print(f"[{name}] 投递/唯一差异={diff_ratio:.2%}"
                      f"（{'疑似重复投递' if diff_ratio >= 0.01 else '一致'}）", flush=True)

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
