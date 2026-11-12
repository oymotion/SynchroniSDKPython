# -*- coding: utf-8 -*-
"""复现 OY App Issues 断连：按 iOS log 的操作序列模拟，触发 half-dead link。

来源 log：
  - Bin Analysis/OY App Issues/Cerelax-Pro_log_20261110_151119.txt     （profile 操作序列）
  - Bin Analysis/OY App Issues/sensor_controller_log_20261110_151110.txt （controller 扫描序列）

设备：Cerelax Pro（BC:93:2A:3F:85:0B / identity 850B）。

本脚本按 log 操作序列复现；为加速复现 half-dead，将 powerRefreshInterval 缩到 1000ms
  （log 原始 10000ms），并按 ENABLE_PPG / ENABLE_EEG / ENABLE_IMU 起流开关设置对应流；其余与 log 一致：
  1) startScan(period=2000ms) -> scan 匹配目标设备 -> stopScan
  2) requireSensor + autoReconnect=False（log: autoReconnect enabled=false）
  3) connect 到 Ready
  4) init(packageSampleCount=16, powerRefreshInterval=1000, timeoutInMS=10000)  （1000 为加速复现，log 原始 10000）
  5) setParam 序列（与 log 顺序一致）：
       DEBUG_BLE_DATA_PATH=...   （iOS 端 BLE 数据落盘；Python 用本地路径，非关键，失败不阻断）
       FILTER_HPF=ON、FILTER_50HZ=ON、FILTER_60HZ=ON、FILTER_LPF=ON
       NTF_EMG=ON                （Cerelax Pro 无 EMG，期望返回 Error）
       NTF_PPG / NTF_EEG / NTF_IMU = ON/OFF（按复现开关，起流前打开/关闭）
  6) getParam 序列（与 log 顺序一致）：
       EEG_SAMPLE_RATE_LIST / EEG_SAMPLE_RATE
       IMU_SAMPLE_RATE_LIST / IMU_SAMPLE_RATE
       PPG_SAMPLE_RATE_LIST / PPG_SAMPLE_RATE
  7) setParam 采样率（仅在对应流开启时设置）：PPG=400 / EEG=1000 / IMU=200
  8) startDataNotification
  9) 观察窗口：检测 half-dead link（起流后命令通道超时 -> SDK 强制断开）

预期现象（对应 iOS log）：
  - NTF_EMG=ON 返回 Error（set_function_switch fail）
  - 起流后约 1s 出现 command timeout(0x08)（interval=1000），约 3s 后 half-dead link 强制断开
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))            # .../Disconnect Issue Debug/
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))      # .../Automation/
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import scan_and_match

# ---- 复现参数（与 iOS log 一致）----
SCAN_PERIOD_MS = 2000                 # log: startScan period=2000ms
PACKAGE_SAMPLE_COUNT = 16             # log: init packageSampleCount=16
POWER_REFRESH_INTERVAL_MS = 500      # 加速复现（log 原始值 10000）
INIT_TIMEOUT_MS = 10000               # log: init timeoutInMS=10000
OBSERVE_SECONDS = 7200                  # 起流后观察时长（log 中约 30~50s 触发 half-dead）

# ---- 起流开关 + 采样率（复现参数：起流前按此 setParam 打开/关闭）----
ENABLE_PPG = True                     # 是否起流 PPG
ENABLE_EEG = False                    # 是否起流 EEG
ENABLE_IMU = False                     # 是否起流 IMU（注意：IMU 需 EEG 同起才有数据，co_stream=NTF_EEG）
PPG_SAMPLE_RATE = "400"               # PPG 采样率（log 原始 100，加速复现提到 400）
EEG_SAMPLE_RATE = "1000"              # EEG 采样率（默认 500，这里 1000）
IMU_SAMPLE_RATE = "200"               # IMU 采样率（默认 200）
PPG_CONFIRM_SECONDS = 3               # 起流后确认数据流的观察时长
BATTERY_STOP_PERCENT = 5              # 电量低于该百分比时停止脚本
BATTERY_CHECK_INTERVAL_SEC = 5        # 观察窗口内查电量的间隔（秒）

# setParam / getParam 顺序（严格对应 log 的调用顺序）
FILTER_SEQ = ["FILTER_HPF", "FILTER_50HZ", "FILTER_60HZ", "FILTER_LPF"]
GETPARAM_SEQ = [
    "EEG_SAMPLE_RATE_LIST", "EEG_SAMPLE_RATE",
    "IMU_SAMPLE_RATE_LIST", "IMU_SAMPLE_RATE",
    "PPG_SAMPLE_RATE_LIST", "PPG_SAMPLE_RATE",
]


def _dt_name(dt):
    try:
        return dt.name
    except Exception:
        return str(dt)


class DataMonitor:
    """统计收到的数据回调，用于确认「data flowing」（half-dead 判定的前提）。"""

    def __init__(self):
        self.count = 0
        self.types = set()
        self.ppg_count = 0
        self.eeg_count = 0
        self.imu_count = 0

    def on_data(self, sensor, data_list):
        items = data_list if isinstance(data_list, list) else [data_list]
        self.count += len(items)
        for d in items:
            try:
                dt = d.getDataType()
                self.types.add(dt)
                if dt == DataType.NTF_PPG:
                    self.ppg_count += 1
                elif dt == DataType.NTF_EEG:
                    self.eeg_count += 1
                elif dt in (DataType.NTF_ACC, DataType.NTF_GYRO):
                    # Cerelax Pro 无 NTF_IMU 独立数据类型，IMU 6 轴按 acc/gyro 两路投递
                    self.imu_count += 1
            except Exception:
                pass


def _log_ts(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _battery_level(sensor):
    """读取电量百分比；失败或未知（返回 <0）视为 None。"""
    try:
        level = sensor.getBatteryLevel()
    except Exception as e:
        _log_ts(f"[电量] getBatteryLevel 抛异常 {type(e).__name__}: {e}")
        return None
    return None if level is None or level < 0 else level


def main():
    ctrl = SensorControllerInstance

    print("=" * 70, flush=True)
    print("复现 OY App Issues 断连：模拟 iOS log 操作序列", flush=True)
    print("=" * 70, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    # ---- 开启 SDK 日志（捕获 SDK 自身的 command timeout / half-dead 消息）----
    log_dir = os.path.join(BASE_DIR, "sdk_logs")
    try:
        ctrl.setLogPath(True, log_dir)
        ctrl.setDebugEnabled(True)
        print(f"[日志] SDK 日志已开启，目录: {log_dir}", flush=True)
    except Exception as e:
        print(f"[日志] 开启 SDK 日志失败（不影响主流程）: {type(e).__name__}: {e}", flush=True)

    if ctrl.isEnable is not True:
        print("[失败] 电脑蓝牙未开启，请先开启后重跑。", flush=True)
        ctrl.terminate()
        return

    # ---- 1) 复现 controller log：startScan(period=2000ms) -> stopScan ----
    try:
        ret = ctrl.startScan(SCAN_PERIOD_MS)
        print(f"[controller] startScan({SCAN_PERIOD_MS}) -> {ret}", flush=True)
    except Exception as e:
        print(f"[controller] startScan({SCAN_PERIOD_MS}) 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[controller] isScanning = {ctrl.isScanning}", flush=True)

    try:
        stop_ret = ctrl.stopScan()
        print(f"[controller] stopScan() -> {stop_ret}", flush=True)
    except Exception as e:
        print(f"[controller] stopScan() 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 2) scan 匹配目标设备 ----
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print(f"[失败] 未匹配到目标设备（identity={config.TARGET_IDENTITY}）。", flush=True)
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

    # ---- 3) autoReconnect=False（log: autoReconnect enabled=false）----
    sensor.autoReconnect = False
    print(f"[profile] autoReconnect 设为 False，读回 = {sensor.autoReconnect}", flush=True)

    # 注册回调：观察状态机 / 错误 / 数据 / 自动重连
    states = []
    errors = []
    reconnects = []

    def on_state(s, st):
        states.append(st)
        _log_ts(f"[onStateChanged] {getattr(s.BLEDevice, 'Name', '?')} -> {st}")

    def on_error(s, reason):
        errors.append(reason)
        _log_ts(f"[onErrorCallback] {getattr(s.BLEDevice, 'Name', '?')}: {reason!r}")

    def on_reconnect(s, restore, answer):
        reconnects.append(restore)
        _log_ts(f"[onAutoReconnect] {getattr(s.BLEDevice, 'Name', '?')} restore={restore}")
        answer(False)

    sensor.onStateChanged = on_state
    sensor.onErrorCallback = on_error
    sensor.onAutoReconnect = on_reconnect

    # ---- 4) connect 到 Ready ----
    _log_ts("[profile] connect() ...")
    try:
        ok = sensor.connect()
        _log_ts(f"[profile] connect() -> {ok}")
    except Exception as e:
        _log_ts(f"[profile] connect() 抛异常 {type(e).__name__}: {e}")
        ctrl.terminate()
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[失败] 未到达 Ready，state={sensor.deviceState}", flush=True)
        ctrl.terminate()
        return
    _log_ts(f"[profile] 到达 Ready")

    # 打印 DeviceInfo（确认 EMG 通道数=0，解释 NTF_EMG=ON 为何失败）
    try:
        info = sensor.getDeviceInfo()
        if info is not None:
            print(f"[DeviceInfo] ModelName={getattr(info, 'ModelName', '?')} "
                  f"Firmware={getattr(info, 'FirmwareVersion', '?')} "
                  f"EmgChannelCount={getattr(info, 'EmgChannelCount', '?')} "
                  f"EegChannelCount={getattr(info, 'EegChannelCount', '?')} "
                  f"PpgChannelCount={getattr(info, 'PpgChannelCount', '?')}", flush=True)
    except Exception as e:
        print(f"[DeviceInfo] 获取失败: {type(e).__name__}: {e}", flush=True)

    # ---- 5) init(packageSampleCount=16, powerRefreshInterval=10000, timeoutInMS=10000) ----
    _log_ts(f"[profile] init({PACKAGE_SAMPLE_COUNT}, {POWER_REFRESH_INTERVAL_MS}, {INIT_TIMEOUT_MS}) ...")
    try:
        iret = sensor.init(PACKAGE_SAMPLE_COUNT, POWER_REFRESH_INTERVAL_MS, INIT_TIMEOUT_MS)
        _log_ts(f"[profile] init() -> {iret}")
    except Exception as e:
        _log_ts(f"[profile] init() 抛异常 {type(e).__name__}: {e}")

    # ---- 6) setParam 序列（严格按 log 顺序）----
    # DEBUG_BLE_DATA_PATH（iOS 端 BLE 数据落盘；Python 用本地路径，失败不阻断）
    ble_log_path = os.path.join(BASE_DIR, "ble_data_log.bin")
    try:
        r = sensor.setParam("DEBUG_BLE_DATA_PATH", ble_log_path)
        _log_ts(f"[setParam] DEBUG_BLE_DATA_PATH={ble_log_path} -> {r!r}")
    except Exception as e:
        _log_ts(f"[setParam] DEBUG_BLE_DATA_PATH 抛异常 {type(e).__name__}: {e}")

    for key in FILTER_SEQ:
        try:
            r = sensor.setParam(key, "ON")
            _log_ts(f"[setParam] {key}=ON -> {r!r}")
        except Exception as e:
            _log_ts(f"[setParam] {key}=ON 抛异常 {type(e).__name__}: {e}")

    # NTF_EMG=ON（Cerelax Pro 无 EMG，期望 Error）
    try:
        r = sensor.setParam("NTF_EMG", "ON")
        _log_ts(f"[setParam] NTF_EMG=ON -> {r!r}  <== 期望 Error（无 EMG 能力）")
    except Exception as e:
        _log_ts(f"[setParam] NTF_EMG=ON 抛异常 {type(e).__name__}: {e}")

    # ---- 起流开关：按复现参数 setParam 打开/关闭 PPG / EEG / IMU ----
    for flag, key, label in (
        (ENABLE_PPG, "NTF_PPG", "PPG"),
        (ENABLE_EEG, "NTF_EEG", "EEG"),
        (ENABLE_IMU, "NTF_IMU", "IMU"),
    ):
        val = "ON" if flag else "OFF"
        try:
            r = sensor.setParam(key, val)
            _log_ts(f"[setParam] {key}={val} -> {r!r}  <== {'起流 ' + label if flag else '关闭 ' + label}")
        except Exception as e:
            _log_ts(f"[setParam] {key}={val} 抛异常 {type(e).__name__}: {e}")

    # ---- 7) getParam 序列（严格按 log 顺序）----
    for key in GETPARAM_SEQ:
        try:
            r = sensor.getParam(key)
            _log_ts(f"[getParam] {key} -> {r!r}")
        except Exception as e:
            _log_ts(f"[getParam] {key} 抛异常 {type(e).__name__}: {e}")

    # ---- 8) setParam 采样率（仅在对应流开启时设置）----
    for flag, key, rate in (
        (ENABLE_PPG, "PPG_SAMPLE_RATE", PPG_SAMPLE_RATE),
        (ENABLE_EEG, "EEG_SAMPLE_RATE", EEG_SAMPLE_RATE),
        (ENABLE_IMU, "IMU_SAMPLE_RATE", IMU_SAMPLE_RATE),
    ):
        if not flag:
            continue
        try:
            r = sensor.setParam(key, rate)
            _log_ts(f"[setParam] {key}={rate} -> {r!r}")
        except Exception as e:
            _log_ts(f"[setParam] {key} 抛异常 {type(e).__name__}: {e}")

    # ---- 起流前电量检查 ----
    batt = _battery_level(sensor)
    _log_ts(f"[电量] 起流前电量 = {batt if batt is not None else '未知'}")
    if batt is not None and batt < BATTERY_STOP_PERCENT:
        _log_ts(f"[电量] 起流前电量 {batt}% < {BATTERY_STOP_PERCENT}%，停止脚本。")
        try:
            sensor.disconnect()
        except Exception:
            pass
        ctrl.terminate()
        return

    # ---- 9) startDataNotification ----
    monitor = DataMonitor()
    sensor.onDataCallback = monitor.on_data
    _log_ts("[profile] startDataNotification() ...")
    start_ts = time.time()
    try:
        sret = sensor.startDataNotification()
        _log_ts(f"[profile] startDataNotification() -> {sret}")
    except Exception as e:
        _log_ts(f"[profile] startDataNotification() 抛异常 {type(e).__name__}: {e}")

    # ---- 起流后确认 PPG / EEG / IMU 是否有数据（起流后短时间应收到通知）----
    time.sleep(PPG_CONFIRM_SECONDS)
    seen_types = ", ".join(_dt_name(t) for t in monitor.types) if monitor.types else "无"
    _log_ts(f"[数据确认] 起流 {PPG_CONFIRM_SECONDS}s 内：PPG={monitor.ppg_count} "
            f"EEG={monitor.eeg_count} IMU(acc+gyro)={monitor.imu_count}；已见类型={seen_types}")

    # ---- 10) 观察窗口：检测 half-dead link ----
    print("\n" + "=" * 70, flush=True)
    print(f"观察窗口 {OBSERVE_SECONDS}s：等待 half-dead link（命令通道超时 -> SDK 强制断开）", flush=True)
    print("=" * 70, flush=True)

    disconnected_at = None
    battery_low_stop = False
    last_state = sensor.deviceState
    t_prev = start_ts
    t_batt = start_ts
    while time.time() - start_ts < OBSERVE_SECONDS:
        time.sleep(0.5)
        elapsed = time.time() - start_ts
        st = sensor.deviceState
        if st != last_state:
            _log_ts(f"[状态] deviceState {last_state} -> {st}（+{elapsed:.1f}s）")
            last_state = st
        if st == DeviceStateEx.Disconnected and disconnected_at is None:
            disconnected_at = elapsed
            _log_ts(f"<== 检测到 Disconnected（起流后 +{elapsed:.1f}s）")
        # 每 BATTERY_CHECK_INTERVAL_SEC 检查电量，低于阈值停止
        if elapsed - t_batt >= BATTERY_CHECK_INTERVAL_SEC:
            t_batt = elapsed
            batt = _battery_level(sensor)
            if batt is not None:
                _log_ts(f"[电量] 当前电量 = {batt}%")
                if batt < BATTERY_STOP_PERCENT:
                    _log_ts(f"[电量] 电量 {batt}% < {BATTERY_STOP_PERCENT}%，停止脚本。")
                    battery_low_stop = True
                    break
        # 每 10s 打印一次数据流状态
        if elapsed - t_prev >= 10:
            seen = ", ".join(_dt_name(t) for t in monitor.types) if monitor.types else "无"
            _log_ts(f"[数据] 已收 {monitor.count} 个数据回调，类型: {seen}")
            t_prev = elapsed

    # ---- 收尾 ----
    try:
        sensor.stopDataNotification()
    except Exception:
        pass
    try:
        sensor.disconnect()
    except Exception:
        pass
    ctrl.terminate()

    # ---- 结论 ----
    print("\n" + "=" * 70, flush=True)
    print("结论", flush=True)
    print("=" * 70, flush=True)
    data_flowing = monitor.count > 0
    print(f"数据流: {'有数据（' + str(monitor.count) + ' 个回调）' if data_flowing else '无数据'}", flush=True)
    print(f"PPG 数据: {'有（' + str(monitor.ppg_count) + ' 个回调）' if monitor.ppg_count > 0 else '无'}", flush=True)
    print(f"EEG 数据: {'有（' + str(monitor.eeg_count) + ' 个回调）' if monitor.eeg_count > 0 else '无'}", flush=True)
    print(f"IMU 数据: {'有（' + str(monitor.imu_count) + ' 个回调）' if monitor.imu_count > 0 else '无（acc+gyro）'}", flush=True)
    print(f"状态序列: {[str(s) for s in states]}", flush=True)
    print(f"错误回调 {len(errors)} 次: {errors!r}", flush=True)
    print(f"自动重连回调 {len(reconnects)} 次", flush=True)

    if battery_low_stop:
        print(f"\n[停止] 电量低于 {BATTERY_STOP_PERCENT}%，提前停止。", flush=True)
    elif data_flowing and disconnected_at is not None:
        print(f"\n[复现] 复现 half-dead link：数据在流但起流后 +{disconnected_at:.1f}s 断开。", flush=True)
        print("       对照 iOS log 中的 command timeout(0x08) -> command channel failure(1/3..3/3)", flush=True)
        print("       -> half-dead link -> forcing disconnect。", flush=True)
        print(f"       SDK 侧详细日志见: {os.path.join(log_dir, '*.txt')}", flush=True)
    elif disconnected_at is not None:
        print(f"\n[部分] 起流后 +{disconnected_at:.1f}s 断开，但未观察到数据流（data flowing 缺失）。", flush=True)
    else:
        print(f"\n[未复现] 观察 {OBSERVE_SECONDS}s 内未断开。可调大 OBSERVE_SECONDS 或多次重跑。", flush=True)


if __name__ == "__main__":
    main()
