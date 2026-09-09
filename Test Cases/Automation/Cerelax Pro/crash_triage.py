# -*- coding: utf-8 -*-
"""Cerelax 头环「死机/复位循环」分阶段排查脚本。

背景：
  现象是设备不断被重启：BLE 扫描能发现，一连上约 2~3 秒即被重启断开。
  本脚本通过「一次连接内分阶段停留观察」，逼近究竟是哪个动作触发掉线：

    阶段1  仅连接（到达 Ready 后保持，不 init、不起流）
    阶段2  init 之后
    阶段3  startDataNotification 起流之后

  每阶段停留 STAGE_SECONDS 秒，持续监测 deviceState 是否离开 Ready（掉线/重启）。
  若某阶段掉线，则本轮把该阶段记为「触发点」并提前结束；可多轮 REPEAT 尝试复现，
  统计各阶段触发次数，辅助判断是「连接即崩」「init 后崩」还是「起流后崩」。

  另附：数据流开关 / 采样率「能力探测」（仅打印结果，不作为判定依据）。

用法：
  python crash_triage.py            # 使用 config.TARGET_IDENTITY（当前 Cerelax 851C）
  python crash_triage.py 850B       # 指定目标 identity（须在 config.DEVICES 中定义）

说明：
  - SDK 当前可能不支持逐路开关 / 采样率设置，本脚本只探测并打印，与排查重点一致。
  - 能力探测（setParam NTF_* ON/OFF）已后置到「停流之后」执行，避免在起流前把所有
    数据流开关误关掉（会导致起流后无任何数据回调，表现为 0 批 / 0 样本）。
  - 起流阶段保持连接+init 后的默认数据流开关状态，不做额外 setParam，与 e2e_smoke 一致。
  - 全程注册 onStateChanged / onErrorCallback / onPowerChanged / onAutoReconnect 回调，
    记录「状态/错误/电量/重连」时间线，用于捕捉掉电复位（BOR）等电量相关重启。
  - 电量读数：阶段1「仅连接」未 init 时 getBatteryLevel 返回 -1 属预期（电量上报由
    init 的 powerRefreshInterval 启动）；init 之后才读到有效值。
  - 通过 ctrl.setLogPath 落盘 SDK 日志，便于事后回看断开原因与最后数据包。
"""

import os
import sys
import time
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match, resolve_target_identity

# ---- 排查参数（按需修改）----
STAGE_SECONDS = 15      # 每阶段停留观察时长（秒），覆盖「连上 2~3 秒断开」
REPEAT = 3              # 完整轮次，用于偶发问题复现
READY_TIMEOUT = 15      # connect 后等待到达 Ready 的超时（秒）

# 逐路开关能力探测候选键（Cerelax 功能：imu/eeg/spo2/ppg/阻抗）
# 仅探测 setParam 是否支持，返回 Error 即记录「不支持」并跳过，不影响主流程。
NTF_PROBE_KEYS = ["NTF_EEG", "NTF_IMU", "NTF_PPG", "NTF_SPO2", "NTF_IMPEDANCE"]


class DataCounter:
    """统计 onDataCallback 收到的批数与样本数（线程安全）。

    calls：回调被触发的次数，用于区分「没回调」还是「回调了但空数据」。
    """

    def __init__(self):
        self.batches = 0
        self.samples = 0
        self.calls = 0
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.calls += 1
            self.batches += len(items)
            for it in items:
                cs = getattr(it, 'channelSamples', None)
                if not cs:
                    continue
                try:
                    self.samples += sum(len(ch) for ch in cs)
                except TypeError:
                    self.samples += len(cs)

    def snapshot(self):
        with self.lock:
            return self.batches, self.samples, self.calls


class EventLog:
    """记录带时间戳的事件（状态/错误/电量/重连），线程安全。"""

    def __init__(self):
        self.events = []
        self.lock = threading.Lock()

    def add(self, kind, detail):
        with self.lock:
            self.events.append((time.time(), kind, detail))

    def snapshot(self):
        with self.lock:
            return list(self.events)

    def dump(self, title):
        print(f"\n{title}", flush=True)
        evs = self.snapshot()
        if not evs:
            print("  （无事件）", flush=True)
            return
        for ts, kind, detail in evs:
            ms = int((ts - int(ts)) * 1000)
            hhmmss = time.strftime('%H:%M:%S', time.localtime(ts))
            print(f"  {hhmmss}.{ms:03d}  [{kind}] {detail}", flush=True)


def _wait_ready(sensor, timeout=READY_TIMEOUT):
    """connect 后等待到达 Ready。返回 (是否到达, 耗时秒)。"""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        st = getattr(sensor, 'deviceState', None)
        if st != last:
            print(f"  [等待Ready] deviceState: {last} -> {st}  (t=+{time.time() - t0:.1f}s)", flush=True)
            last = st
        if st == DeviceStateEx.Ready:
            return True, time.time() - t0
        time.sleep(0.2)
    return False, timeout


def _monitor_stage(sensor, label, seconds, counter=None):
    """在已 Ready 的前提下，停留观察 seconds 秒。

    返回 (掉线, 详情)。掉线判定：deviceState 离开 Ready。
    """
    print(f"\n[阶段·{label}] 已 Ready，停留观察 {seconds}s ...", flush=True)
    t0 = time.time()
    last = None
    next_print = 0.0
    dropped = False
    drop_t = None

    while time.time() - t0 < seconds:
        st = getattr(sensor, 'deviceState', None)
        if st != last:
            print(f"  [{label}] deviceState: {last} -> {st}  (t=+{time.time() - t0:.1f}s)", flush=True)
            last = st

        if st is not None and st != DeviceStateEx.Ready:
            dropped = True
            drop_t = time.time() - t0
            print(f"  [{label}] ⚠ 离开 Ready 状态（可能掉线/重启）: {st}  (t=+{drop_t:.1f}s)", flush=True)
            break

        if time.time() - t0 >= next_print:
            parts = [f"t=+{time.time() - t0:.0f}s"]
            if counter is not None:
                b, s, c = counter.snapshot()
                parts.append(f"回调{c}次/{b}批/{s}样本")
            batt = _read_battery(sensor)
            batt_txt = f"{batt}" if batt != -1 else "-1（无有效读数，未 init 前属预期）"
            parts.append(f"电量={batt_txt}")
            print(f"  [{label}] " + " | ".join(parts), flush=True)
            next_print = time.time() - t0 + 2.0

        time.sleep(0.2)

    if dropped:
        detail = f"阶段「{label}」在 t=+{drop_t:.1f}s 掉线（state={last}）"
    else:
        detail = f"阶段「{label}」持续 {seconds}s 未掉线"
    return dropped, detail


def _read_battery(sensor):
    """读取电量，返回 int 或错误字符串。"""
    try:
        return sensor.getBatteryLevel()
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _register_callbacks(sensor, events):
    """注册状态/错误/电量/重连回调，事件写入 EventLog。

    电量回调只入 event（上报较频繁），其余即时打印。
    """

    def on_state(s, st):
        events.add("state", f"{st}")
        print(f"  [状态回调] -> {st}", flush=True)

    def on_error(s, reason):
        events.add("error", f"{reason}")
        print(f"  [错误回调] -> {reason}", flush=True)

    def on_power(s, level):
        events.add("power", f"{level}")

    def on_reconnect(*args):
        events.add("reconnect", f"{args}")
        print(f"  [重连回调] -> {args}", flush=True)

    for attr, cb in [("onStateChanged", on_state),
                     ("onErrorCallback", on_error),
                     ("onPowerChanged", on_power),
                     ("onAutoReconnect", on_reconnect)]:
        try:
            setattr(sensor, attr, cb)
        except Exception as e:
            print(f"  [回调] 注册 {attr} 失败: {type(e).__name__}: {e}", flush=True)


def _probe_stream_switches(sensor):
    """探测逐路数据流开关与采样率能力，仅打印，不改变主流程判定。"""
    print("\n[能力探测] 逐路数据流开关 / 采样率（仅打印，结果不影响判定）", flush=True)

    for key in NTF_PROBE_KEYS:
        try:
            on = sensor.setParam(key, "ON")
        except Exception as e:
            on = f"抛异常 {type(e).__name__}: {e}"
        try:
            off = sensor.setParam(key, "OFF")
        except Exception as e:
            off = f"抛异常 {type(e).__name__}: {e}"
        supported = (on == "OK") and (off == "OK")
        print(f"  setParam('{key}', ON/OFF) -> on={on!r} off={off!r} "
              f"{'支持' if supported else '不支持/返回Error'}", flush=True)

    for key in ["EEG_SAMPLE_RATE", "EEG_SAMPLE_RATE_LIST"]:
        try:
            v = sensor.getParam(key)
        except Exception as e:
            v = f"抛异常 {type(e).__name__}: {e}"
        print(f"  getParam('{key}') -> {v!r}", flush=True)


def _connect(sensor):
    """connect 并等待 Ready。返回 (是否成功, 说明)。"""
    try:
        ok = sensor.connect()
        txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {txt}", flush=True)

    ready, elapsed = _wait_ready(sensor)
    if not ready:
        return False, f"connect 后 {READY_TIMEOUT}s 内未到达 Ready（连接阶段即掉线/连不上）"
    return True, f"connect 后 {elapsed:.1f}s 到达 Ready"


def main():
    ctrl = SensorControllerInstance
    target_identity = resolve_target_identity()

    print("=" * 60, flush=True)
    print("Cerelax 死机/复位循环 分阶段排查", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"观察时长/阶段 = {STAGE_SECONDS}s，轮次 = {REPEAT}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：Cerelax 头环 上电、在范围内", flush=True)

    # SDK 日志落盘，便于回看断开原因
    log_dir = os.path.join(BASE_DIR, "crash_logs")
    os.makedirs(log_dir, exist_ok=True)
    try:
        ctrl.setLogPath(True, log_dir)
        ctrl.setDebugEnabled(True)
        print(f"\n[日志] 已开启，落盘目录 {log_dir}", flush=True)
    except Exception as e:
        print(f"[日志] 开启失败（忽略）: {type(e).__name__}: {e}", flush=True)

    # 环境检查
    if ctrl.isEnable is not True:
        print("[跳过] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    if target_identity:
        print(f"\n[扫描] 目标 identity: {', '.join(target_identity)}（命令行指定）", flush=True)
    else:
        print(f"\n[扫描] 目标 identity: {', '.join(common.TARGET_IDENTITIES)}（config 默认）", flush=True)

    round_results = []

    for rnd in range(1, REPEAT + 1):
        print(f"\n{'#' * 60}\n第 {rnd}/{REPEAT} 轮\n{'#' * 60}", flush=True)

        target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
        if target is None:
            print("[跳过本轮] 未匹配到目标设备", flush=True)
            round_results.append((rnd, "未匹配到设备", "未扫描到目标"))
            continue

        name = getattr(target, 'Name', '?')
        addr = getattr(target, 'Address', '?')
        print(f"[扫描] 目标设备: {name} {addr}", flush=True)

        sensor = ctrl.requireSensor(target)
        if sensor is None:
            print("[跳过本轮] requireSensor 返回 None", flush=True)
            round_results.append((rnd, "requireSensor 失败", "返回 None"))
            continue

        # 注册状态/错误/电量/重连回调，全程记录事件（含电量时间线）
        events = EventLog()
        _register_callbacks(sensor, events)

        # ---- 阶段1：仅连接 ----
        ok, connect_txt = _connect(sensor)
        print(f"[阶段·1-仅连接] {connect_txt}", flush=True)
        if not ok:
            round_results.append((rnd, "1-仅连接(连接阶段)", connect_txt))
            try:
                sensor.disconnect()
            except Exception:
                pass
            continue

        dropped, detail = _monitor_stage(sensor, "1-仅连接", STAGE_SECONDS)
        if dropped:
            round_results.append((rnd, "1-仅连接", detail))
            try:
                sensor.disconnect()
            except Exception:
                pass
            continue

        # ---- init ----
        print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
        try:
            iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
            init_txt = f"返回 {iret} hasInited={sensor.hasInited}"
        except Exception as e:
            iret = None
            init_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[init] {init_txt}", flush=True)

        # ---- 阶段2：init 后 ----
        dropped, detail = _monitor_stage(sensor, "2-init后", STAGE_SECONDS)
        if dropped:
            round_results.append((rnd, "2-init后", detail))
            try:
                sensor.disconnect()
            except Exception:
                pass
            continue

        # ---- 起流 ----
        counter = DataCounter()
        sensor.onDataCallback = counter
        print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
        try:
            sret = sensor.startDataNotification()
            start_txt = f"返回 {sret}"
        except Exception as e:
            sret = None
            start_txt = f"抛异常 {type(e).__name__}: {e}"
        print(f"[起流] {start_txt}  isDataTransfering={sensor.isDataTransfering}", flush=True)

        # ---- 阶段3：起流后 ----
        dropped, detail = _monitor_stage(sensor, "3-起流后", STAGE_SECONDS, counter=counter)
        b, s, calls = counter.snapshot()
        detail += f"（回调 {calls} 次 / {b} 批 / {s} 样本）"
        if dropped:
            round_results.append((rnd, "3-起流后", detail))
        else:
            round_results.append((rnd, "全程未掉线", detail))

        # ---- 清理 ----
        try:
            sensor.stopDataNotification()
        except Exception:
            pass

        # ---- 能力探测（后置，避免污染起流阶段）----
        _probe_stream_switches(sensor)

        try:
            sensor.disconnect()
        except Exception:
            pass

        events.dump(f"第 {rnd} 轮事件时间线（状态/错误/电量/重连）")

        time.sleep(1.0)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("排查汇总", flush=True)
    print("=" * 60, flush=True)
    for rnd, stage, detail in round_results:
        print(f"  第{rnd}轮 -> 触发点/结果: {stage} | {detail}", flush=True)

    from collections import Counter
    stage_count = Counter(stage for _, stage, _ in round_results)
    print("\n各阶段触发/结果统计:", flush=True)
    for stage, n in stage_count.items():
        print(f"  {stage}: {n} 次", flush=True)

    print("\n提示：单轮「全程未掉线」不代表设备健康，仅说明本轮未复现。", flush=True)
    print("     若某阶段多次触发，即为重点怀疑路径（连接/init/起流）。", flush=True)
    print(f"\n[日志] SDK 日志落盘目录: {log_dir}", flush=True)

    ctrl.terminate()


if __name__ == "__main__":
    main()
