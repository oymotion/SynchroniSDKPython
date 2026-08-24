# -*- coding: utf-8 -*-
"""Longevity 长稳：长时间起流，确保 bin 落盘 + debug log 打开。

运行策略：
  - 连接后开启 setLogPath + setDebugEnabled(True) + DEBUG_LOG_PATH=True + DEBUG_BLE_DATA_PATH=True
  - startDataNotification 持续起流，周期打印电量/数据吞吐/运行时长
  - 退出条件（满足其一即停）：
      1) 运行时长 >= MIN_DURATION_SECONDS（默认 5 小时）
      2) 电量 < LOW_BATTERY_THRESHOLD（默认 5%）
      3) 数据流意外中断（isDataTransfering=False）或设备状态异常（未保持 Ready）
  - 结束时 stopDataNotification + disconnect，导出 bin，并校验 bin 文件存在
  - 支持 Ctrl+C 中断，同样走收尾流程

任何退出路径（包括连接/init/起流失败）都统一走 _teardown() 收尾，
保证 log 目录与 bin 是否存在被打印出来，便于事后定位。

前置条件：
  - 主机(电脑)：蓝牙已开启、电源稳定（长时间运行建议插电）
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import sys
import time
import tempfile
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(BASE_DIR)
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import _identity_of, scan_and_match

MIN_DURATION_SECONDS = 5 * 3600  # 最小运行时长（秒），默认 5 小时
LOW_BATTERY_THRESHOLD = 5        # 电量低于该百分比则退出
CHECK_INTERVAL = 60              # 每 60 秒输出一次状态
BIN_COMPLETENESS_RATIO = 0.99    # bin 录制时长/实际起流时长 的完整度下界
STALL_CHECK_CYCLES = 3           # 连续 N 个周期 batches 无增长即判数据停滞

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

def _list_log_files(log_dir):
    """列出日志目录中所有 .log 和 .txt 文件，返回 {filename: fullpath}。"""
    if not log_dir or not os.path.isdir(log_dir):
        return {}
    out = {}
    try:
        for fn in os.listdir(log_dir):
            if fn.lower().endswith((".log", ".txt")):
                out[fn] = os.path.join(log_dir, fn)
    except OSError:
        pass
    return out

class DataCounter:
    def __init__(self):
        self.batches = 0
        self.samples = 0
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.batches += len(items)
            for it in items:
                try:
                    self.samples += it.getSampleCount() or 0
                except Exception:
                    pass

    def snapshot(self):
        with self.lock:
            return self.batches, self.samples

def _teardown(ctrl, sensor, log_dir, bins_before, stream_seconds=None):
    """统一收尾：尝试停流/断开、导出并校验 bin、打印 log 目录，并清理资源。

    任何退出路径都调用本函数，确保 log 目录和 bin 是否存在能被打印出来。
    stream_seconds 为实际起流时长（秒），用于校验 bin 录制完整性；None 表示未起流，跳过完整性校验。
    返回 (have_bin, bin_path, bin_valid, bin_complete)：
      bin_valid     : getBinFileInfo 返回非 None（bin 可回放/可解析）
      bin_complete  : replay_duration >= stream_seconds * BIN_COMPLETENESS_RATIO；无法判定时 None
    """
    ble_path = None
    if sensor is not None:
        try:
            sensor.stopDataNotification()
        except Exception as e:
            print(f"[收尾] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
        try:
            ble_path = sensor.getParam("DEBUG_BLE_DATA_PATH")
        except Exception as e:
            print(f"[收尾] getParam('DEBUG_BLE_DATA_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)
        try:
            sensor.disconnect()
        except Exception as e:
            print(f"[收尾] disconnect 抛异常 {type(e).__name__}: {e}", flush=True)
        # 断开后再取一次路径，导出发生在断开之后
        if not (isinstance(ble_path, str) and ble_path.strip()):
            try:
                ble_path = sensor.getParam("DEBUG_BLE_DATA_PATH")
            except Exception:
                pass

    time.sleep(0.5)

    bins_after = _list_bins(log_dir)
    new_bins = sorted(set(bins_after.keys()) - set(bins_before.keys()))
    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if bin_path is None and new_bins:
        bin_path = bins_after[new_bins[0]]
    have_bin = bool(bin_path) and os.path.isfile(bin_path)

    print(f"[收尾] log 目录: {log_dir}", flush=True)
    print(f"[收尾] getParam('DEBUG_BLE_DATA_PATH') = {ble_path!r}", flush=True)
    print(f"[收尾] 新增 bin 文件: {new_bins if new_bins else '无'}", flush=True)
    print(f"[收尾] bin 文件存在 = {have_bin}（{bin_path!r}）", flush=True)

    # ---- bin 数据质量判断：有效性（可解析）+ 完整性（录制时长达标）----
    bin_valid = False
    bin_complete = None
    if have_bin:
        try:
            bin_info = ctrl.getBinFileInfo(bin_path)
        except Exception as e:
            bin_info = None
            print(f"[收尾] getBinFileInfo 抛异常 {type(e).__name__}: {e}", flush=True)
        bin_valid = bin_info is not None
        print(f"[收尾] bin 有效性 = {'有效' if bin_valid else '无效'}（getBinFileInfo 返回 {'dict' if bin_valid else 'None'}）", flush=True)

        if bin_valid and stream_seconds:
            try:
                replay_duration = bin_info.get("replay_duration")
            except Exception:
                replay_duration = None
            if replay_duration is not None:
                try:
                    ratio = float(replay_duration) / float(stream_seconds)
                    bin_complete = ratio >= BIN_COMPLETENESS_RATIO
                    print(f"[收尾] bin 完整性 = {'完整' if bin_complete else '不完整'}（replay_duration={replay_duration}s / 起流 {stream_seconds:.1f}s = {ratio:.1%}）", flush=True)
                except (TypeError, ValueError):
                    print(f"[收尾] bin 完整性 = 无法判定（replay_duration={replay_duration!r}）", flush=True)
            else:
                print("[收尾] bin 完整性 = 无法判定（无 replay_duration 字段）", flush=True)

    if sensor is not None:
        try:
            sensor.setParam("DEBUG_BLE_DATA_PATH", "False")
        except Exception:
            pass
        try:
            sensor.setParam("DEBUG_LOG_PATH", "False")
        except Exception:
            pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass
    try:
        ctrl.terminate()
    except Exception:
        pass

    return have_bin, bin_path, bin_valid, bin_complete

def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("Longevity 长稳：长时间起流 + bin 落盘 + debug log", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"最小运行时长: {MIN_DURATION_SECONDS}s（{MIN_DURATION_SECONDS / 3600:.1f} 小时）", flush=True)
    print(f"低电量退出阈值: {LOW_BATTERY_THRESHOLD}%", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启、建议插电", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "本脚本将长时间运行，完成后按回车开始 ...")

    # 受控日志目录（固定前缀，便于结束后定位）
    log_dir = tempfile.mkdtemp(prefix="sdklog_longevity_")
    print(f"\n[日志目录] 使用受控目录 {log_dir}", flush=True)
    log_files_before = _list_log_files(log_dir)
    try:
        ctrl.setLogPath(True, log_dir)
    except Exception as e:
        print(f"[日志目录] setLogPath 抛异常 {type(e).__name__}: {e}", flush=True)

    # log 打开校验：setDebugEnabled(True) 后应在日志目录创建 .log/.txt 文件
    log_opened = False
    try:
        ctrl.setDebugEnabled(True)
        print("[日志] SensorController.setDebugEnabled(True) 已开启", flush=True)
        time.sleep(2.0)
        log_files_after = _list_log_files(log_dir)
        new_logs = sorted(set(log_files_after.keys()) - set(log_files_before.keys()))
        log_opened = bool(new_logs) or (bool(log_files_after) and not log_files_before)
        if log_opened:
            print(f"[日志] 校验通过：已创建日志文件 {list(log_files_after.keys())}", flush=True)
        else:
            print("[日志] 校验失败：setDebugEnabled(True) 后日志目录中未发现 .log/.txt 文件", flush=True)
    except Exception as e:
        print(f"[日志] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    bins_before = _list_bins(log_dir)

    sensor = None

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到目标设备，退出", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return
    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None，退出", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return

    # connect
    try:
        ok = sensor.connect()
    except Exception as e:
        ok = None
        print(f"[连接] connect 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[连接] SensorProfile.connect() -> {ok}  state={sensor.deviceState}", flush=True)
    if ok is not True:
        print("[FAIL] connect 失败，退出", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"[FAIL] 未到达 Ready（state={sensor.deviceState}），退出", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return

    # init
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
    except Exception as e:
        iret = None
        print(f"[init] init 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[init] SensorProfile.init() -> {iret}  hasInited={sensor.hasInited}", flush=True)
    if iret is not True:
        print("[FAIL] init 失败，退出", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return

    # 开启 profile 日志与 bin 导出
    try:
        sensor.setParam("DEBUG_LOG_PATH", "True")
        print("[日志] SensorProfile.setParam('DEBUG_LOG_PATH', 'True') 已开启", flush=True)
    except Exception as e:
        print(f"[日志] setParam('DEBUG_LOG_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
        print(f"[bin] SensorProfile.setParam('DEBUG_BLE_DATA_PATH', 'True') -> {bret!r}", flush=True)
    except Exception as e:
        print(f"[bin] setParam('DEBUG_BLE_DATA_PATH') 抛异常 {type(e).__name__}: {e}", flush=True)

    # 电量回调
    lock = threading.Lock()
    latest_power = [None]

    def on_power_changed(sensor, level):
        with lock:
            latest_power[0] = level

    sensor.onPowerChanged = on_power_changed

    # 数据回调
    counter = DataCounter()
    sensor.onDataCallback = counter

    # 起流
    try:
        sret = sensor.startDataNotification()
    except Exception as e:
        sret = None
        print(f"[起流] startDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[起流] SensorProfile.startDataNotification() -> {sret}  isDataTransfering={sensor.isDataTransfering}", flush=True)
    if sret is not True:
        print("[FAIL] 起流失败，退出", flush=True)
        _teardown(ctrl, sensor, log_dir, bins_before)
        return

    # ---- 长稳循环 ----
    start_time = time.time()
    exit_reason = ""
    last_batches = None      # 上一周期 batches，用于数据停滞检测
    stall_cycles = 0         # 连续无增长的周期数
    power_unknown_warned = False  # 电量 -1 是否已写错误日志

    def _fmt_hms(sec):
        h = int(sec // 3600)
        m = int((sec % 3600) // 60)
        s = int(sec % 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    try:
        while True:
            elapsed = time.time() - start_time
            with lock:
                power = latest_power[0]
            if power is None:
                try:
                    power = sensor.getBatteryLevel()
                except Exception:
                    power = None

            # 电量 -1 兜底：写入错误日志（仅首次，避免每周期刷屏）
            if power is None or power == -1:
                if not power_unknown_warned:
                    try:
                        ctrl.log(f"[Longevity] 电量未知（power={power!r}），低电量退出条件将失效，仅依赖时长退出", "E")
                    except Exception as e:
                        print(f"[电量] ctrl.log 写错误日志抛异常 {type(e).__name__}: {e}", flush=True)
                    print(f"[电量] 未知（power={power!r}），已写入错误日志；低电量退出条件失效", flush=True)
                    power_unknown_warned = True

            # 退出条件 1：电量低于阈值
            if isinstance(power, int) and 0 <= power < LOW_BATTERY_THRESHOLD:
                exit_reason = f"电量 {power}% < {LOW_BATTERY_THRESHOLD}%"
                break

            # 退出条件 2：达到最小运行时长
            if elapsed >= MIN_DURATION_SECONDS:
                exit_reason = f"达到最小运行时长 {MIN_DURATION_SECONDS}s"
                break

            # 退出条件 3：数据流/设备状态异常
            if sensor.isDataTransfering is not True:
                exit_reason = f"数据流意外中断（isDataTransfering={sensor.isDataTransfering}）"
                break
            if sensor.deviceState != DeviceStateEx.Ready:
                exit_reason = f"设备状态异常（deviceState={sensor.deviceState}）"
                break

            batches, samples = counter.snapshot()

            # 退出条件 4：数据停滞（连续 STALL_CHECK_CYCLES 个周期 batches 无增长）
            if last_batches is not None and batches == last_batches:
                stall_cycles += 1
                if stall_cycles >= STALL_CHECK_CYCLES:
                    exit_reason = (f"数据停滞：连续 {STALL_CHECK_CYCLES} 个周期"
                                   f"（约 {STALL_CHECK_CYCLES * CHECK_INTERVAL}s）batches 无增长（batches={batches}）")
                    break
            else:
                stall_cycles = 0
            last_batches = batches

            print(f"[状态] {_fmt_hms(elapsed)}  电量={power}%  数据={batches}批/{samples}样本", flush=True)
            time.sleep(CHECK_INTERVAL)
    except KeyboardInterrupt:
        exit_reason = "用户手动中断（Ctrl+C）"

    print(f"\n[退出] 原因: {exit_reason}  总运行时长 {_fmt_hms(time.time() - start_time)}", flush=True)

    stream_seconds = time.time() - start_time
    have_bin, bin_path, bin_valid, bin_complete = _teardown(ctrl, sensor, log_dir, bins_before, stream_seconds=stream_seconds)

    print("\n" + "=" * 60, flush=True)
    print("Longevity 结果", flush=True)
    print("=" * 60, flush=True)
    print(f"  退出原因: {exit_reason}", flush=True)
    print(f"  运行时长: {_fmt_hms(stream_seconds)}", flush=True)
    print(f"  日志目录: {log_dir}", flush=True)
    print(f"  debug 日志: {'已打开' if log_opened else '未打开'}", flush=True)
    print(f"  bin 文件: {'存在' if have_bin else '缺失'}（{bin_path!r}）", flush=True)
    if have_bin:
        print(f"  bin 有效性: {'有效' if bin_valid else '无效'}", flush=True)
        if bin_complete is None:
            print("  bin 完整性: 无法判定（无 replay_duration 或数据不足）", flush=True)
        else:
            print(f"  bin 完整性: {'完整' if bin_complete else '不完整'}", flush=True)

    if not have_bin:
        verdict = "FAIL（未生成 bin）"
    elif not log_opened:
        verdict = "FAIL（debug 日志未打开）"
    elif not bin_valid:
        verdict = "FAIL（bin 无效，无法回放/解析）"
    elif bin_complete is False:
        verdict = "FAIL（bin 录制不完整）"
    else:
        verdict = "PASS"
    print("\n结论: " + verdict, flush=True)

if __name__ == "__main__":
    main()
