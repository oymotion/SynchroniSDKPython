# -*- coding: utf-8 -*-
"""BIN-FUNC-007：实时起流中拒绝回放。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-007
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) 生成有效 bin（connect → 起流 → 采集 → stop → disconnect）
  2) 重新 connect → 到达 Ready → init → startDataNotification 起流
  3) 起流过程中调用 replayBinFile(bin_path, sensor, realtime=True)
  4) 断言：回放被拒绝（返回 None 或抛异常）

说明：
  README：replayBinFile 在目标 sensor 正在实时起流时应拒绝回放，状态互斥。
  本用例先生成一个有效 bin，再重新连接起流，在起流中尝试回放，验证互斥。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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

COLLECT_SECONDS = 5  # 起流采集时长（秒），用于生成 bin
SETTLE_SECONDS = 2   # 停流后的刹车时间（秒）


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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-007 实时起流中拒绝回放", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
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

    # ---- 阶段 1：生成 bin ----
    print("\n[阶段1] 生成 bin 文件 ...", flush=True)

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
    record(results, "startDataNotification 返回 True", sret is True,
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
        record(results, "起流中回放被拒绝", None, "回放返回 None 或抛异常", "无有效 bin")
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

    # ---- 阶段 2：重新连接起流，在起流中尝试回放，验证录制不受影响 ----
    print("\n[阶段2] 重新连接，起流中尝试回放，验证录制继续 ...", flush=True)

    print("\n[连接] SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
        connect2_txt = f"返回 {ok}"
    except Exception as e:
        ok = None
        connect2_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[连接] SensorProfile.connect() -> {connect2_txt}  state={sensor.deviceState}", flush=True)
    record(results, "阶段2 connect 返回 True", ok is True,
           "connect() 返回 True", f"connect() -> {connect2_txt}")

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    ready2 = (sensor.deviceState == DeviceStateEx.Ready)
    record(results, "阶段2 到达 Ready", ready2, "deviceState==Ready", f"state={sensor.deviceState}")

    if not ready2:
        print("[FAIL] 阶段2 未到达 Ready，无法继续", flush=True)
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
        init2_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init2_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] init() -> {init2_txt}", flush=True)
    record(results, "阶段2 init 返回 True", iret is True, "init() 返回 True", f"init() -> {init2_txt}")

    # 阶段 2 也开启 bin 导出，以便后续验证 bin 时长覆盖了回放拒绝后的时间
    print("\n[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') ...", flush=True)
    try:
        bret2 = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
    except Exception as e:
        bret2 = f"抛异常 {type(e).__name__}: {e}"
    print(f"[bin] setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret2!r}", flush=True)
    record(results, "阶段2 setParam('DEBUG_BLE_DATA_PATH', 'True') 返回 OK", bret2 == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret2!r}")

    # 注册回调，追踪回放拒绝前后的数据量
    class Stage2Counter:
        def __init__(self):
            self.count = 0
        def __call__(self, sensor, data):
            items = data if isinstance(data, list) else [data]
            self.count += len(items)
    counter = Stage2Counter()
    sensor.onDataCallback = counter

    print("\n[起流] SensorProfile.startDataNotification() ...", flush=True)
    try:
        sret = sensor.startDataNotification()
        start2_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        start2_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[起流] startDataNotification() -> {start2_txt}", flush=True)
    record(results, "阶段2 startDataNotification 返回 True", sret is True,
           "startDataNotification() 返回 True", f"startDataNotification() -> {start2_txt}")

    streaming = sensor.isDataTransfering
    print(f"[起流] isDataTransfering = {streaming}", flush=True)
    record(results, "起流后 isDataTransfering == True", streaming is True,
           "isDataTransfering == True", f"isDataTransfering == {streaming}")

    # 等待数据开始流动，记录回放前的数据量和时间
    time.sleep(1)
    before_replay_count = counter.count
    before_replay_ts = time.time()
    print(f"[回放前] 数据批数 = {before_replay_count}，时间戳 = {before_replay_ts:.0f}", flush=True)

    # ---- 核心断言 1：起流中调回放应被拒绝 ----
    print(f"\n[回放] 起流中尝试 replayBinFile({bin_path!r}, sensor, realtime=True) ...", flush=True)
    replay_result = None
    replay_error = None
    try:
        replay_result = ctrl.replayBinFile(bin_path, sensor, realtime=True)
    except Exception as e:
        replay_error = f"{type(e).__name__}: {e}"

    rejected = (replay_result is None) or (replay_error is not None)
    print(f"[回放] 返回={replay_result!r}  异常={replay_error}", flush=True)
    record(results, "起流中回放被拒绝（返回 None 或抛异常）", rejected,
           "replayBinFile 返回 None 或抛异常",
           f"返回={replay_result!r} 异常={replay_error}")

    # ---- 核心断言 2：回放被拒绝后，录制继续（数据仍在增长）----
    print(f"\n[回放后] 继续采集 {COLLECT_SECONDS}s，验证录制仍在继续 ...", flush=True)
    time.sleep(COLLECT_SECONDS)
    after_replay_count = counter.count
    after_replay_ts = time.time()
    growth = after_replay_count - before_replay_count
    print(f"[回放后] 数据批数 = {after_replay_count}（增长 {growth}）", flush=True)
    record(results, "回放被拒绝后数据仍在增长（录制继续）", growth > 0,
           "回放后采集数据批数 > 回放前", f"增长 {growth}")

    # 停止录制
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    ble_path2 = _get_ble_path(sensor)
    print(f"[bin] stop 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path2!r}", flush=True)

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    if not isinstance(ble_path2, str) or not ble_path2.strip():
        ble_path2 = _get_ble_path(sensor)
        print(f"[bin] disconnect 后 getParam('DEBUG_BLE_DATA_PATH') = {ble_path2!r}", flush=True)

    time.sleep(0.5)

    bins_after2 = _list_bins(log_dir)
    new_bins2 = sorted(set(bins_after2.keys()) - set(bins_before.keys()) - set(new_bins))
    print(f"\n[检查] 阶段2 新增 .bin 文件: {new_bins2 if new_bins2 else '无'}", flush=True)

    bin_path2 = ble_path2 if (isinstance(ble_path2, str) and ble_path2.strip()) else None
    if bin_path2 is None and new_bins2:
        bin_path2 = bins_after2[new_bins2[0]]

    have_bin2 = bool(bin_path2) and os.path.isfile(bin_path2)
    print(f"[检查] 阶段2 bin 路径: {bin_path2!r}，文件存在={have_bin2}", flush=True)

    # ---- 核心断言 3：bin 时长覆盖了回放拒绝后的时间 ----
    if have_bin2:
        try:
            info = ctrl.getBinFileInfo(bin_path2)
        except Exception as e:
            info = None
            print(f"[bin] getBinFileInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        if isinstance(info, dict):
            duration = info.get("replay_duration", 0)
            streaming_duration = after_replay_ts - before_replay_ts
            print(f"[bin] 阶段2 bin replay_duration = {duration:.3f}s，回放前→回放后+采集 = {streaming_duration:.1f}s", flush=True)
            # bin 时长应覆盖回放前到停止的完整时段（至少 ≥ 回放后采集时长）
            duration_ok = isinstance(duration, (int, float)) and duration >= streaming_duration
            record(results, "bin 时长覆盖回放拒绝后的录制时段", duration_ok,
                   f"bin replay_duration >= {streaming_duration:.1f}s",
                   f"replay_duration={duration:.3f}s")
        else:
            record(results, "bin 时长覆盖回放拒绝后的录制时段", None,
                   "getBinFileInfo 返回有效 dict", "getBinFileInfo 返回 None 或非 dict")
    else:
        record(results, "bin 时长覆盖回放拒绝后的录制时段", None,
               "阶段2 生成有效 bin", "无阶段2 bin")

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