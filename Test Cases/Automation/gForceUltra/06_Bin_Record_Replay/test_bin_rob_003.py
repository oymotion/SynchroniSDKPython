# -*- coding: utf-8 -*-
"""BIN-ROB-003：回放控制重复调用幂等。

对应用例：06_Bin录制回放解析.md -> BIN-ROB-003
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) 生成有效 bin（connect → 起流 → 采集 30s → stop → disconnect）
  2) 未回放时依次调用 pauseBinReplay/resumeBinReplay/stopBinReplay，校验不崩溃
  3) 在子线程中 replayBinFile(path, sensor, realtime=True)
  4) 回放中重复调用 pauseBinReplay（两次），校验不崩溃
  5) 重复调用 resumeBinReplay（两次），校验不崩溃
  6) stopBinReplay 停止；停止后再次 stopBinReplay，校验不崩溃

说明：
  README：pauseBinReplay/resumeBinReplay/stopBinReplay 返回 "OK" 或错误字符串。
  幂等语义：重复调用、状态不符时调用，均应被明确处理（返回 OK 或拒绝），不崩溃、不抛异常。
  本用例只断言"不崩溃、不抛异常"，不强制每次返回 "OK"（状态不符时允许返回错误字符串）。
  录制 30s + realtime=True 按原始节奏回放，给重复 pause/resume/stop 留出充足操作窗口。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：gForceUltra 上电、在范围内
"""

import json
import os
import subprocess
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

COLLECT_SECONDS = 30  # 起流采集时长（秒）：录制足够长，确保回放有充足窗口执行重复控制


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


def _on_error(sensor, reason):
    print(f"[onErrorCallback] {getattr(sensor, 'BLEDevice', None)}: {reason}", flush=True)


def _run_worker(bin_path, mode, realtime=False, timeout=180):
    """在独立子进程里回放，返回解析后的 JSON dict。"""
    worker = os.path.join(AUTOMATION_DIR, "replay_worker.py")
    cmd = [sys.executable, worker, "--bin", bin_path, "--mode", mode]
    if mode == "summary":
        cmd += ["--realtime", "true" if realtime else "false"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        lines = [ln for ln in (proc.stdout or "").splitlines() if ln.strip()]
        if not lines:
            return {"ok": False, "error": "子进程无 stdout",
                    "returncode": proc.returncode, "stderr": (proc.stderr or "")[:500]}
        try:
            return json.loads(lines[-1])
        except Exception as e:
            return {"ok": False, "error": f"解析子进程 JSON 失败 {type(e).__name__}: {e}",
                    "stdout": (proc.stdout or "")[:500], "stderr": (proc.stderr or "")[:500]}
    except Exception as e:
        return {"ok": False, "error": f"子进程回放异常 {type(e).__name__}: {e}"}


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-ROB-003 回放控制重复调用幂等", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：gForceUltra 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 gForceUltra 已【开机】且在范围内，"
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
        record(results, "未回放时控制调用不崩溃", None, "不抛异常", "无有效 bin")
        record(results, "回放中重复 pause 不崩溃", None, "不抛异常", "无有效 bin")
        record(results, "回放中重复 resume 不崩溃", None, "不抛异常", "无有效 bin")
        record(results, "停止后再次 stop 不崩溃", None, "不抛异常", "无有效 bin")
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

    # ---- 清理并终止本进程 SDK（释放 dongle，避免与回放子进程冲突）----
    try:
        sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
    except Exception:
        pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass

    ctrl.terminate()

    # ---- 回放控制幂等：独立子进程（fresh controller + fresh profile）----
    # 进程内回放会复用已连接/已起流的 profile（0 批/0 样本），故必须子进程回放。
    result = _run_worker(bin_path, "idempotency")
    r_ok = result.get("ok") is True
    r_err = result.get("error")

    if r_ok:
        detail = {k: result.get(k) for k in
                  ('no_replay_crash', 'flowed', 'pause_crash_free',
                   'resume_crash_free', 'stop_crash_free', 'thread_done')}
        print(f"[回放] 子进程幂等回放完成：{detail}", flush=True)
    else:
        print(f"[回放] 子进程幂等回放失败：{r_err}", flush=True)

    record(results, "未回放时控制调用不崩溃", r_ok and result.get("no_replay_crash") is True,
           "不抛异常", f"errs={result.get('no_replay_errs')}")

    record(results, "回放开始产生数据", r_ok and result.get("flowed") is True,
           "进入回放后 count > 0", f"flowed={result.get('flowed')}")

    record(results, "回放中重复 pause 不崩溃", r_ok and result.get("pause_crash_free") is True,
           "不抛异常", f"errs={result.get('pause_errs')}")

    record(results, "回放中重复 resume 不崩溃", r_ok and result.get("resume_crash_free") is True,
           "不抛异常", f"errs={result.get('resume_errs')}")

    record(results, "停止后再次 stop 不崩溃", r_ok and result.get("stop_crash_free") is True,
           "不抛异常", f"errs={result.get('stop_errs')}")

    thread_done = result.get("thread_done")
    record(results, "stop 后回放子线程结束", r_ok and thread_done is True,
           "子进程回放线程在超时内结束",
           f"thread_done={thread_done}" + (f" err={result.get('replay_error')}" if result.get('replay_error') else ""))

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
