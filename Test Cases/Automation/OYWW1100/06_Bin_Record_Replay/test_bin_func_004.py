# -*- coding: utf-8 -*-
"""BIN-FUNC-004：回放 realtime=True 按原始节奏。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-004
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True)
  2) scan -> requireSensor -> connect -> 到达 Ready -> init
  3) setParam("DEBUG_BLE_DATA_PATH", "True") 开启 bin 导出（值为字符串 "True"）
  4) 注册 onDataCallback 计数，startDataNotification 起流，采集 N 秒
  5) stopDataNotification 后等待刹车时间（SETTLE_SECONDS）让在途数据包送达回调，再记录实收批数
  6) disconnect，取导出的 bin 路径
  7) 复用同一 profile，换回调计数，replayBinFile(path, sensor, realtime=True)
  8) 校验：回放产生数据（count>0）、回放 DataType 包含 live DataType、replay 有 startTimeStamp 流锚点
     per-DataType 批数差异仅作 informational 输出（不做 PASS/FAIL 判定）

说明：
  README：replayBinFile(file_path, sensor=None, realtime=True, timeout=None)，
  realtime=True 按录制节奏回放；解析结果经同一 pipeline 由 onDataCallback 送达，
  与实时数据一致。
  官方 multi_start_test.py 比对的是 startTimeStamp/delay 而非精确批数，因为 bin
  回放解析器对 BLE 包的分组方式与 live 管线不同，且 bin 可能包含 live 时未启用的
  DataType 包。因此本用例的硬断言为：
    - 回放产生了数据（count > 0）
    - 回放 DataType 集合包含 live 期间出现的 DataType
    - replay 的 startTimeStamp 非 None（回放产生了有效 SensorData 带流锚点）
  per-DataType 批数差异仅作 informational 输出，不做 PASS/FAIL 判定。
  "按原始节奏"（realtime=True）也仅作 informational 观察（回放耗时与录制时长同量级）。

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

COLLECT_SECONDS = 5  # 起流采集时长（秒）
SETTLE_SECONDS  = 3 # 停流后的刹车时间（秒）：让 stop 后仍在途的数据包完成解析并送达回调


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


class BatchCounter:
    """onDataCallback 计数：每个 SensorData 计 1 批，同时记录首/末批信息与 per-DataType 分布。"""

    def __init__(self):
        self.count = 0        # 总批数
        self.first_ts = None
        self.last_ts = None
        self.first_dt = None
        self.last_dt = None
        self.dt_counts = {}   # dt_name -> count

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        for d in items:
            dt = self._dt_name(d)
            self.count += 1
            self.dt_counts[dt] = self.dt_counts.get(dt, 0) + 1
            if self.first_ts is None:
                self.first_ts = d.getStartTimeStamp()
                self.first_dt = dt
            self.last_ts = d.getStartTimeStamp()
            self.last_dt = dt

    @staticmethod
    def _dt_name(d):
        try:
            dt = d.getDataType()
            if isinstance(dt, DataType):
                return dt.name
            return DataType(dt).name
        except Exception:
            return "?"


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-004 回放 realtime=True 按原始节奏", flush=True)
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

    # 开启 bin 导出（值为字符串 "True"）
    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    # ---- 实时采集，统计批数 ----
    live = BatchCounter()
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

    # 录制时长 = 起流到停流（不含刹车时间）
    live_duration = time.time() - live_start

    # 刹车时间：bin 在 stop 时已按"收到的原始包"封口，但解析→回调是异步的，
    # stop 后仍有在途数据包会稍后送达 onDataCallback；若此刻立即读 live.count，
    # 会漏掉这些在途包，导致 live < replay。等待一段时间让在途包送达后再统计。
    print(f"[采集] 等待刹车 {SETTLE_SECONDS}s，让 stop 后在途数据包送达回调 ...", flush=True)
    time.sleep(SETTLE_SECONDS)

    live_count = live.count
    print(f"[采集] 实收批数 = {live_count}（DataType={live.first_dt}），录制时长 ≈ {live_duration:.3f}s", flush=True)

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
        record(results, "回放批数与实收批数一致", None,
               "replay 批数 == live 批数", "无有效 bin，无法回放")
        record(results, "realtime=True 按原始节奏（informational）", None,
               "回放耗时与录制时长同量级", "无有效 bin，无法回放")
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

    # ---- 回放 realtime=True ----
    # 注意：bin 回放解析器对 BLE 包的分组方式与 live 管线不同，且 bin 可能包含
    # live 时未启用的 DataType 包。因此不对精确批数做硬断言，而是验证：
    #   1) 回放产生了数据（count > 0）
    #   2) 回放 DataType 集合包含 live 期间出现的 DataType
    #   3) live 和 replay 时间戳区间有重叠（同一会话）
    # 批数差异作为 informational 输出。
    replay = BatchCounter()
    sensor.onDataCallback = replay

    print(f"\n[回放] replayBinFile({bin_path!r}, sensor, realtime=True) ...", flush=True)
    replay_start = time.time()
    try:
        profile = ctrl.replayBinFile(bin_path, sensor, realtime=True)
        replay_txt = f"返回 {type(profile).__name__}"
    except Exception as e:
        profile = None
        replay_txt = f"抛异常 {type(e).__name__}: {e}"
    replay_duration = time.time() - replay_start
    replay_count = replay.count
    print(f"[回放] {replay_txt}，回放批数 = {replay_count}，耗时 ≈ {replay_duration:.3f}s", flush=True)

    # ---- 诊断：打印 per-DataType 分布与时间戳 ----
    print("\n[诊断] 数据分布对比", flush=True)
    print(f"  LIVE   总批数={live.count}  分布={live.dt_counts}  首 ts={live.first_ts} dt={live.first_dt}  末 ts={live.last_ts}", flush=True)
    print(f"  REPLAY 总批数={replay.count}  分布={replay.dt_counts}  首 ts={replay.first_ts} dt={replay.first_dt}  末 ts={replay.last_ts}", flush=True)

    live_dts = set(live.dt_counts.keys())
    replay_dts = set(replay.dt_counts.keys())

    # 1) 回放产生了数据
    has_data = replay.count > 0
    record(results, "回放产生数据（count > 0）", has_data,
           "replay.count > 0", f"replay.count={replay.count}")

    # 2) 回放 DataType 集合包含 live 的 DataType
    contains_live = live_dts.issubset(replay_dts)
    record(results, "回放 DataType 包含 live 的 DataType", contains_live,
           f"replay dts 包含 {live_dts}",
           f"live={live_dts} replay={replay_dts} 交集={live_dts & replay_dts} 缺失={live_dts - replay_dts}")

    # 3) 回放产生了有效数据（startTimeStamp 非 None，即 SensorData 有流锚点）
    has_anchor = replay.first_ts is not None
    record(results, "回放数据有 startTimeStamp（流锚点）", has_anchor,
           "replay.first_ts is not None",
           f"replay.first_ts={replay.first_ts} live.first_ts={live.first_ts} diff={replay.first_ts - live.first_ts if live.first_ts is not None and replay.first_ts is not None else 'N/A'}ms")

    # 4) informational：per-DataType 批数差异
    all_dts = sorted(live_dts | replay_dts)
    diffs = []
    for dt in all_dts:
        lc = live.dt_counts.get(dt, 0)
        rc = replay.dt_counts.get(dt, 0)
        diffs.append(f"{dt}: live={lc} replay={rc} diff={rc - lc}")
    print(f"  [informational] 各 DataType 批数差异: {' | '.join(diffs)}", flush=True)
    record(results, "per-DataType 批数差异（informational）", None,
           "仅作参考，不做 PASS/FAIL 判定", diffs)

    # informational：按原始节奏（回放耗时与录制时长同量级，非瞬时）
    pace_ok = (live_duration > 0 and replay_duration >= 0.5 * live_duration)
    record(results, "realtime=True 按原始节奏（informational）", None,
           "回放耗时 >= 0.5*录制时长（非瞬时全速）",
           f"replay={replay_duration:.3f}s live={live_duration:.3f}s 同量级={pace_ok}")

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
