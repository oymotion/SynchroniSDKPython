# -*- coding: utf-8 -*-
"""BIN-FUNC-009：回放还原 startTimeStamp/delay 与实收一致。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-009
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True)
  2) scan -> requireSensor -> connect -> 到达 Ready -> init
  3) setParam("DEBUG_BLE_DATA_PATH", "True") 开启 bin 导出
  4) 注册 onDataCallback，startDataNotification 起流，采集 N 秒
  5) 采集期间记录最后一批有效数据的 (startTimeStamp, delay)
  6) stopDataNotification + 刹车等待 + disconnect，取 bin 路径
  7) 换回调，replayBinFile(path, sensor, realtime=False) 全速回放
  8) 回放期间记录首批有效数据的 (startTimeStamp, delay)
  9) 校验：回放还原的 (startTimeStamp, delay) 与实收一致（tuple 严格相等）

说明：
  参考 multi_start_test.py 的比对逻辑：
  - LIVE 端取最后一批有效数据（startTimeStamp 非 None/0），持续覆盖以收敛到起流会话的值
  - REPLAY 端取首批有效数据（startTimeStamp 非 None/0）
  - 比对 tuple (startTimeStamp, delay)，要求严格一致
  getStartTimeStamp() 返回 session 级锚点（同一流内所有批次相同），
  getDelay() 返回首原始包 bin 记录时间戳与 startTimeStamp 的差值（ms）。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import re
import sys
import time
import tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

COLLECT_SECONDS = 5   # 起流采集时长（秒）
SETTLE_SECONDS  = 2   # 停流后的刹车时间（秒）


def _list_bins(log_dir):
    if not log_dir or not os.path.isdir(log_dir):
        return {}
    out = {}
    try:
        for fn in os.listdir(log_dir):
            if fn.lower().endswith(".bin"):
                out[fn] = os.path.join(log_dir, fn)
    except OSError:
        pass
    return out


def _get_ble_path(sensor):
    try:
        return sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


class LiveCollector:
    """LIVE 端回调：记录最后一批有效数据的 (startTimeStamp, delay)。

    参考 multi_start_test.py 的 LIVE 端逻辑：持续覆盖以收敛到起流会话的值。
    """

    def __init__(self):
        self.count = 0
        self.first_ts = None
        self.first_delay = None
        self.last_ts = None
        self.last_delay = None

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.count += 1
            ts = d.getStartTimeStamp()
            delay = d.getDelay()
            if ts:
                if self.first_ts is None:
                    self.first_ts = ts
                    self.first_delay = delay
                self.last_ts = ts
                self.last_delay = delay

    @property
    def live_pair(self):
        """返回最后一批有效数据的 (startTimeStamp, delay)，无有效数据时返回 None。"""
        if self.last_ts is not None:
            return (self.last_ts, self.last_delay)
        return None


class ReplayCollector:
    """REPLAY 端回调：记录首批有效数据的 (startTimeStamp, delay)。

    参考 multi_start_test.py 的 REPLAY 端逻辑：取首个有效批。
    """

    def __init__(self):
        self.count = 0
        self.first_ts = None
        self.first_delay = None

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            self.count += 1
            ts = d.getStartTimeStamp()
            delay = d.getDelay()
            if ts and self.first_ts is None:
                self.first_ts = ts
                self.first_delay = delay

    @property
    def replay_pair(self):
        """返回首批有效数据的 (startTimeStamp, delay)，无有效数据时返回 None。"""
        if self.first_ts is not None:
            return (self.first_ts, self.first_delay)
        return None


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-009 回放还原 startTimeStamp/delay 与实收一致", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    log_dir = tempfile.mkdtemp(prefix="sdklog_bin_")
    print(f"\n[日志目录] 使用受控目录 {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
        log_ok = True
        log_txt = f"setLogPath(True, {log_dir}) 无异常"
    except Exception as e:
        log_ok = False
        log_txt = f"setLogPath 抛异常 {type(e).__name__}: {e}"
    print(f"[日志目录] {log_txt}", flush=True)
    record(results, "setLogPath 设置受控日志目录", log_ok,
           "setLogPath(True, dir) 无异常", log_txt)

    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志目录] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    bins_before = _list_bins(log_dir)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    print(f"[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    print(f"[扫描] 扫描到 {len(devices) if devices else 0} 台设备:", flush=True)
    if devices:
        for d in devices:
            n = getattr(d, 'Name', '?')
            a = getattr(d, 'Address', '?')
            print(f"  {n} {a} identity={_identity_of(n)}", flush=True)
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

    sensor.onErrorCallback = _on_error

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

    # 开启 bin 导出
    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    # ---- LIVE 采集：记录 (startTimeStamp, delay) ----
    live = LiveCollector()
    sensor.onDataCallback = live

    print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    record(results, "SensorProfile.startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start_txt}")

    live_start = time.time()
    print(f"\n[采集] 等待 {COLLECT_SECONDS}s ...", flush=True)
    time.sleep(COLLECT_SECONDS)

    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    live_duration = time.time() - live_start

    print(f"[采集] 等待刹车 {SETTLE_SECONDS}s，让 stop 后在途数据包送达回调 ...", flush=True)
    time.sleep(SETTLE_SECONDS)

    live_pair = live.live_pair
    print(f"[采集] 实收批数 = {live.count}，录制时长 ≈ {live_duration:.3f}s", flush=True)
    print(f"[采集] LIVE  (startTimeStamp, delay) = {live_pair}", flush=True)

    ble_path = _get_ble_path(sensor)
    print(f"[bin] stop 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    if not isinstance(ble_path, str) or not ble_path.strip():
        ble_path = _get_ble_path(sensor)
        print(f"[bin] disconnect 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)

    time.sleep(0.5)

    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    print(f"\n[检查] 新增 .bin 文件: {new_bins if new_bins else '无'}", flush=True)

    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]

    have_bin = bool(bin_path) and os.path.isfile(bin_path)
    print(f"[检查] bin 路径: {bin_path!r}，文件存在={have_bin}", flush=True)
    record(results, "生成有效 bin 供回放使用", have_bin,
           "存在可用 bin 文件", f"{bin_path!r}（存在={have_bin}）")

    if not have_bin:
        record(results, "回放还原 (startTimeStamp, delay) 与实收一致", None,
               "replay (ts, delay) == live (ts, delay)", "无有效 bin，无法回放")
        try:
            sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
        except Exception:
            pass
        try:
            ctrl.setDebugEnabled(False)
        except Exception:
            pass
        ctrl.terminate()
        print("\n结论: FAIL", flush=True)
        return

    # ---- REPLAY：记录首批 (startTimeStamp, delay) ----
    replay = ReplayCollector()
    sensor.onDataCallback = replay

    print(f"\n[回放] replayBinFile({bin_path!r}, sensor, realtime=False) ...", flush=True)
    replay_start = time.time()
    try:
        profile = ctrl.replayBinFile(bin_path, sensor, realtime=False)
        replay_txt = f"返回 {type(profile).__name__}"
    except Exception as e:
        profile = None
        replay_txt = f"抛异常 {type(e).__name__}: {e}"
    replay_duration = time.time() - replay_start
    replay_pair = replay.replay_pair
    print(f"[回放] {replay_txt}，回放批数 = {replay.count}，耗时 ≈ {replay_duration:.3f}s", flush=True)
    print(f"[回放] REPLAY (startTimeStamp, delay) = {replay_pair}", flush=True)

    # ---- 比对 (startTimeStamp, delay) ----
    if live_pair is not None and replay_pair is not None:
        match = (live_pair == replay_pair)
        record(results, "回放还原 (startTimeStamp, delay) 与实收一致",
               match,
               f"replay {replay_pair} == live {live_pair}",
               f"live={live_pair} replay={replay_pair} match={match}")

        ts_diff = abs(live_pair[0] - replay_pair[0]) if live_pair[0] is not None and replay_pair[0] is not None else None
        print(f"  [诊断] startTimeStamp: live={live_pair[0]} replay={replay_pair[0]} diff={ts_diff}ms", flush=True)
        print(f"  [诊断] delay:         live={live_pair[1]} replay={replay_pair[1]}", flush=True)
    elif live_pair is None:
        record(results, "回放还原 (startTimeStamp, delay) 与实收一致", False,
               "live 端有有效数据", "live 端无有效 (startTimeStamp, delay) 数据")
    else:
        record(results, "回放还原 (startTimeStamp, delay) 与实收一致", False,
               "replay 端有有效数据", "replay 端无有效 (startTimeStamp, delay) 数据")

    # 清理
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
    except Exception:
        pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass

    ctrl.terminate()

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        elif status == "SKIP":
            print(f"  [SKIP] {rname}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status == "FAIL":
            all_pass = False

    print(f"\n[提示] 本次 bin 落盘目录: {log_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()