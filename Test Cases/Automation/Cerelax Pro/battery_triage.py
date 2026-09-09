# -*- coding: utf-8 -*-
"""Cerelax 头环「电量/充电/电池状态」专项排查脚本。

背景：
  怀疑死机/复位循环与电量、充电、电池状态有关（如低电压触发 BOR 掉电复位，
  或起流瞬间多路传感器拉电流导致 MCU 欠压复位）。
  本脚本在起流负载下长时间监测电量，捕捉三类信号：

    1. 掉电/电量骤降（相邻读数大幅下降）
    2. 异常读数（-1、超出 0~100、跳变）
    3. 复位/断连事件（onStateChanged 离开 Ready、onErrorCallback、onAutoReconnect）

  并尝试探测 SDK 是否有充电状态相关接口（getParam 探测，无则记录）。

用法：
  python battery_triage.py          # 使用 config.TARGET_IDENTITY
  python battery_triage.py 850B     # 指定 identity

说明：
  - 起流是为了增加负载，让电量变化 / 掉电更明显。
  - 电量数据有两个来源：onPowerChanged 回调（init 后周期上报）与主动 getBatteryLevel。
  - 通过 ctrl.setLogPath 落盘 SDK 日志，便于回看断开原因。
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
from common import _identity_of, scan_and_match, resolve_target_identity

# ---- 排查参数（按需修改）----
OBSERVE_SECONDS = 120        # 起流后监测时长（秒）；怀疑低电量触发可调大
MONITOR_INTERVAL = 2         # 主动 getBatteryLevel 间隔（秒）
READY_TIMEOUT = 15           # connect 后等待 Ready 超时（秒）
POWER_DROP_THRESHOLD = 20    # 掉电判定：相邻读数下降 >= 20%

# 充电状态探测候选 key（SDK 若无此能力，getParam 会返回 Error，仅记录）
CHARGE_PROBE_KEYS = [
    "CHARGING", "CHARGE_STATUS", "CHARGING_STATUS",
    "BATTERY_STATUS", "POWER_STATUS", "BATTERY_CHARGING",
]


class PowerTracker:
    """记录电量时间序列，线程安全。source: 'cb'=onPowerChanged, 'poll'=主动读。"""

    def __init__(self):
        self.samples = []
        self.lock = threading.Lock()

    def add(self, ts, level, source):
        with self.lock:
            self.samples.append((ts, level, source))

    def snapshot(self):
        with self.lock:
            return list(self.samples)


class EventLog:
    """记录带时间戳的事件（状态/错误/重连），线程安全。"""

    def __init__(self):
        self.events = []
        self.lock = threading.Lock()

    def add(self, kind, detail):
        with self.lock:
            self.events.append((time.time(), kind, detail))

    def snapshot(self):
        with self.lock:
            return list(self.events)


def _read_battery(sensor):
    try:
        return sensor.getBatteryLevel()
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def _register_callbacks(sensor, tracker, events):
    def on_power(s, level):
        tracker.add(time.time(), level, "cb")

    def on_state(s, st):
        events.add("state", f"{st}")
        print(f"  [状态回调] -> {st}", flush=True)

    def on_error(s, reason):
        events.add("error", f"{reason}")
        print(f"  [错误回调] -> {reason}", flush=True)

    def on_reconnect(*args):
        events.add("reconnect", f"{args}")
        print(f"  [重连回调] -> {args}", flush=True)

    for attr, cb in [("onPowerChanged", on_power),
                     ("onStateChanged", on_state),
                     ("onErrorCallback", on_error),
                     ("onAutoReconnect", on_reconnect)]:
        try:
            setattr(sensor, attr, cb)
        except Exception as e:
            print(f"  [回调] 注册 {attr} 失败: {type(e).__name__}: {e}", flush=True)


def _probe_charge_status(sensor):
    """探测充电状态相关 getParam 能力，仅打印。"""
    print("\n[充电状态探测] 尝试 getParam 读取充电/电池相关 key（不支持则返回 Error）", flush=True)
    for key in CHARGE_PROBE_KEYS:
        try:
            v = sensor.getParam(key)
        except Exception as e:
            v = f"抛异常 {type(e).__name__}: {e}"
        print(f"  getParam('{key}') -> {v!r}", flush=True)


def main():
    ctrl = SensorControllerInstance
    target_identity = resolve_target_identity()

    print("=" * 60, flush=True)
    print("Cerelax 电量/充电/电池状态 专项排查", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"监测时长 = {OBSERVE_SECONDS}s，主动读电量间隔 = {MONITOR_INTERVAL}s", flush=True)
    print(f"掉电判定阈值 = 相邻读数下降 >= {POWER_DROP_THRESHOLD}%", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：Cerelax 头环 上电、在范围内", flush=True)
    print("  - 建议：先记录当前是否【充电中】/ 电量高低，便于对照分析", flush=True)

    log_dir = os.path.join(BASE_DIR, "battery_logs")
    os.makedirs(log_dir, exist_ok=True)
    try:
        ctrl.setLogPath(True, log_dir)
        ctrl.setDebugEnabled(True)
        print(f"\n[日志] 已开启，落盘目录 {log_dir}", flush=True)
    except Exception as e:
        print(f"[日志] 开启失败（忽略）: {type(e).__name__}: {e}", flush=True)

    if ctrl.isEnable is not True:
        print("[跳过] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    if target_identity:
        print(f"\n[扫描] 目标 identity: {', '.join(target_identity)}（命令行指定）", flush=True)
    else:
        print(f"\n[扫描] 目标 identity: {', '.join(common.TARGET_IDENTITIES)}（config 默认）", flush=True)

    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        ctrl.terminate()
        return

    tracker = PowerTracker()
    events = EventLog()
    _register_callbacks(sensor, tracker, events)

    # connect
    try:
        ok = sensor.connect()
        print(f"[连接] connect() -> {ok}", flush=True)
    except Exception as e:
        print(f"[连接] connect 抛异常 {type(e).__name__}: {e}", flush=True)
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[FAIL] 未到达 Ready（state={sensor.deviceState}）", flush=True)
        ctrl.terminate()
        return
    print(f"[连接] 到达 Ready（{time.time() - t0:.1f}s）", flush=True)

    # init
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        print(f"[init] init() -> {iret}  hasInited={sensor.hasInited}", flush=True)
    except Exception as e:
        print(f"[init] init 抛异常 {type(e).__name__}: {e}", flush=True)
        ctrl.terminate()
        return

    # 初始电量
    init_batt = _read_battery(sensor)
    tracker.add(time.time(), init_batt, "poll")
    print(f"[电量] 初始电量 = {init_batt}", flush=True)

    # 起流（增加负载）
    try:
        sret = sensor.startDataNotification()
        print(f"[起流] startDataNotification() -> {sret}  isDataTransfering={sensor.isDataTransfering}", flush=True)
    except Exception as e:
        print(f"[起流] startDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    # 长时间监测
    print(f"\n[监测] 起流中监测 {OBSERVE_SECONDS}s（电量回调 + 主动读）...", flush=True)
    start = time.time()
    next_poll = start
    dropped = False
    drop_t = None

    while time.time() - start < OBSERVE_SECONDS:
        st = getattr(sensor, 'deviceState', None)
        if st is not None and st != DeviceStateEx.Ready:
            dropped = True
            drop_t = time.time() - start
            print(f"\n⚠ [监测] 离开 Ready 状态: {st}（t=+{drop_t:.1f}s）——疑似掉电/复位！", flush=True)
            break

        if time.time() >= next_poll:
            batt = _read_battery(sensor)
            tracker.add(time.time(), batt, "poll")
            print(f"  [监测] t=+{time.time() - start:.0f}s  电量={batt}", flush=True)
            next_poll = time.time() + MONITOR_INTERVAL

        time.sleep(0.2)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception:
        pass

    # 充电状态探测（连接态下 getParam 才有效）
    _probe_charge_status(sensor)

    # 断开
    try:
        sensor.disconnect()
    except Exception:
        pass

    # ---- 汇总分析 ----
    samples = tracker.snapshot()
    print("\n" + "=" * 60, flush=True)
    print("电量监测汇总", flush=True)
    print("=" * 60, flush=True)

    # 电量样本时间线（仅打印 onPowerChanged 与 poll 的数值）
    if samples:
        print(f"共 {len(samples)} 个电量样本：", flush=True)
        levels = [lv for _, lv, _ in samples]
        valid = [lv for lv in levels if isinstance(lv, int) and 0 <= lv <= 100]
        if valid:
            print(f"  有效读数(0~100): 最小={min(valid)} 最大={max(valid)}", flush=True)

        # 异常读数
        abnormal = [(ts, lv) for ts, lv, _ in samples if not isinstance(lv, int) or lv < 0 or lv > 100]
        print(f"  异常读数(-1/越界/非int): {len(abnormal)} 个", flush=True)
        for ts, lv in abnormal[:10]:
            print(f"    {time.strftime('%H:%M:%S', time.localtime(ts))} -> {lv}", flush=True)

        # 掉电检测（相邻有效读数下降超阈值）
        drops = []
        prev_lv = None
        for ts, lv, src in samples:
            if isinstance(lv, int) and 0 <= lv <= 100:
                if prev_lv is not None and (prev_lv - lv) >= POWER_DROP_THRESHOLD:
                    drops.append((ts, prev_lv, lv, src))
                prev_lv = lv
        print(f"\n[掉电检测] 相邻读数下降 >= {POWER_DROP_THRESHOLD}%: {len(drops)} 次", flush=True)
        for ts, p, c, src in drops[:10]:
            print(f"    {time.strftime('%H:%M:%S', time.localtime(ts))}  {p} -> {c}（{src}）", flush=True)
    else:
        print("未收到任何电量样本（onPowerChanged 与 getBatteryLevel 均无数据）", flush=True)

    # 复位/断连事件
    evs = events.snapshot()
    print(f"\n[复位/断连事件] 共 {len(evs)} 条：", flush=True)
    if evs:
        for ts, kind, detail in evs:
            print(f"    {time.strftime('%H:%M:%S', time.localtime(ts))}  [{kind}] {detail}", flush=True)
    else:
        print("  （无状态/错误/重连事件）", flush=True)

    print("\n结论提示：", flush=True)
    print("  - 若观察到「电量骤降」紧跟着「离开 Ready/断连/重连」→ 高度怀疑掉电复位（BOR/欠压）。", flush=True)
    print("  - 若电量稳定但仍有断连 → 更可能是看门狗/崩溃，而非供电。", flush=True)
    print("  - 充电中 vs 电池供电建议分别跑一次做对照。", flush=True)
    print(f"\n[日志] SDK 日志落盘目录: {log_dir}", flush=True)

    ctrl.terminate()


if __name__ == "__main__":
    main()
