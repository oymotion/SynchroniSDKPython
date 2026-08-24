# -*- coding: utf-8 -*-
"""BIN-FUNC-006：pause/resume/stop 回放。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-006
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) 生成有效 bin（connect → 起流 → 采集 30s → stop → disconnect）
  2) 在子线程中启动 replayBinFile(path, sensor, realtime=True) 按原始节奏回放
  3) 主线程等待数据开始流动，记录当前批数
  4) 调用 pauseBinReplay(sensor)，校验返回 "OK"，等待 3s 后验证批数不再增长
  5) 调用 resumeBinReplay(sensor)，校验返回 "OK"，等待 3s 后验证批数恢复增长
  6) 调用 stopBinReplay(sensor)，校验返回 "OK"，等待子线程结束

说明：
  README：pauseBinReplay/resumeBinReplay/stopBinReplay 均返回 "OK" 或错误字符串。
  replayBinFile 是阻塞调用，因此需要在子线程中执行，主线程负责控制。
  录制 30s + realtime=True 按原始节奏回放，给 pause/resume/stop 留出充足操作窗口。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import re
import sys
import time
import tempfile
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

COLLECT_SECONDS = 30  # 起流采集时长（秒）：录制足够长，确保回放有充足窗口执行 pause/resume/stop
SETTLE_SECONDS = 2  # 停流后的刹车时间（秒）
PAUSE_SLEEP = 3  # pause 后观察窗口（秒）
PAUSE_GROWTH_TOLERANCE = 20  # pause 后"刹车滑行"允许的最大批数增长（在途/缓冲数据仍会短暂流入）
RESUME_SLEEP = 3  # resume 后观察窗口（秒）
THREAD_JOIN_TIMEOUT = 60  # 等待回放子线程结束的超时（秒）


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
    """onDataCallback 计数，线程安全。"""

    def __init__(self):
        self.count = 0
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.count += len(items)

    def snapshot(self):
        with self.lock:
            return self.count


def _call_ctrl(method, *args):
    """调用 controller 方法并返回 (返回值, 异常信息)。"""
    try:
        r = method(*args)
        return r, None
    except Exception as e:
        return None, f"抛异常 {type(e).__name__}: {e}"


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-006 pause/resume/stop 回放", flush=True)
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

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

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

    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

    # ---- 实时采集，生成 bin ----
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

    print(f"\n[采集] 等待 {COLLECT_SECONDS}s ...", flush=True)
    time.sleep(COLLECT_SECONDS)

    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

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
        record(results, "pauseBinReplay 返回 OK", None, "返回 'OK'", "无有效 bin")
        record(results, "pause 后批数停止增长", None, "暂停后批数不变", "无有效 bin")
        record(results, "resumeBinReplay 返回 OK", None, "返回 'OK'", "无有效 bin")
        record(results, "resume 后批数恢复增长", None, "恢复后批数增加", "无有效 bin")
        record(results, "stopBinReplay 返回 OK", None, "返回 'OK'", "无有效 bin")
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

    # ---- 回放控制测试 ----
    counter = BatchCounter()
    sensor.onDataCallback = counter

    replay_error = [None]

    def replay_thread():
        try:
            ctrl.replayBinFile(bin_path, sensor, realtime=True)
        except Exception as e:
            replay_error[0] = f"{type(e).__name__}: {e}"

    print(f"\n[回放] 在子线程中启动 replayBinFile(realtime=True, 录制 {COLLECT_SECONDS}s) ...", flush=True)
    t = threading.Thread(target=replay_thread, daemon=True)
    t.start()

    # 等待数据开始流动（轮询，最多等 5s）
    t_wait = time.time()
    before_pause = 0
    while time.time() - t_wait < 5:
        before_pause = counter.snapshot()
        if before_pause > 0:
            break
        time.sleep(0.2)
    print(f"[回放] 当前批数 = {before_pause}（等待数据流动耗时 {time.time() - t_wait:.1f}s）", flush=True)
    record(results, "回放开始产生数据", before_pause > 0,
           "进入回放后 count > 0", f"before_pause={before_pause}")

    # ---- pause ----
    r, err = _call_ctrl(ctrl.pauseBinReplay, sensor)
    print(f"[pause] pauseBinReplay -> {r!r}  err={err}", flush=True)
    pause_ok = (r == "OK")
    record(results, "pauseBinReplay 返回 OK", pause_ok,
           "返回 'OK'", f"返回 {r!r}  err={err}")

    time.sleep(PAUSE_SLEEP)
    during_pause = counter.snapshot()
    paused_growth = during_pause - before_pause
    print(f"[pause] 暂停 {PAUSE_SLEEP}s 后批数 = {during_pause}（增长 {paused_growth}）", flush=True)
    stopped = paused_growth <= PAUSE_GROWTH_TOLERANCE
    record(results, "pause 后批数停止增长", stopped,
           f"pause 后批数增长 <= {PAUSE_GROWTH_TOLERANCE}", f"增长 {paused_growth}")

    # ---- resume ----
    r, err = _call_ctrl(ctrl.resumeBinReplay, sensor)
    print(f"[resume] resumeBinReplay -> {r!r}  err={err}", flush=True)
    resume_ok = (r == "OK")
    record(results, "resumeBinReplay 返回 OK", resume_ok,
           "返回 'OK'", f"返回 {r!r}  err={err}")

    time.sleep(RESUME_SLEEP)
    after_resume = counter.snapshot()
    resumed_growth = after_resume - during_pause
    print(f"[resume] resume {RESUME_SLEEP}s 后批数 = {after_resume}（增长 {resumed_growth}）", flush=True)
    resumed = resumed_growth > 0
    record(results, "resume 后批数恢复增长", resumed,
           "resume 后批数增长 > 0", f"增长 {resumed_growth}")

    # ---- stop ----
    r, err = _call_ctrl(ctrl.stopBinReplay, sensor)
    print(f"[stop] stopBinReplay -> {r!r}  err={err}", flush=True)
    stop_ok = (r == "OK")
    record(results, "stopBinReplay 返回 OK", stop_ok,
           "返回 'OK'", f"返回 {r!r}  err={err}")

    # 等待子线程结束
    t.join(timeout=THREAD_JOIN_TIMEOUT)
    thread_done = not t.is_alive()
    record(results, "stop 后回放子线程结束", thread_done,
           f"子线程在 {THREAD_JOIN_TIMEOUT}s 内结束",
           f"子线程{'已结束' if thread_done else '仍在运行'}" + (f" err={replay_error[0]}" if replay_error[0] else ""))

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