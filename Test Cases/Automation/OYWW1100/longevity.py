# -*- coding: utf-8 -*-
"""Longevity 长稳（USB dongle 后端）：长时间起流，确保 bin 落盘 + debug log 打开。

连接方式：使用 USB dongle（bumble 后端），不使用系统蓝牙（bleak）。
  - 提示用户关闭系统蓝牙，避免走系统蓝牙通道
  - 运行前用 checkSetupDongle() 校验 dongle 就绪（返回 "OK: N"）
  - 说明：Windows 上 getBLEBackendName() 恒返回 "bleak"（不可靠），
    故不以其作为后端判定，仅以 checkSetupDongle() 结果为准（与官方 demo 一致）

用法：
  python longevity.py            # 使用 config.TARGET_IDENTITY 指定的设备
  python longevity.py 80E5       # 指定目标 identity（须在 config.DEVICES 中定义）

运行策略：
  - 连接后开启 setLogPath + setDebugEnabled(True) + DEBUG_LOG_PATH=True + DEBUG_BLE_DATA_PATH=True
  - startDataNotification 持续起流，周期打印电量/数据吞吐/运行时长
  - 退出条件（满足其一即停）：
      1) 运行时长 >= MIN_DURATION_SECONDS（默认 5 小时）
      2) 电量 < LOW_BATTERY_THRESHOLD（默认 5%）
      3) 数据流意外中断（isDataTransfering=False）或设备状态异常（未保持 Ready），
         但会先进入 RECONNECT_GRACE_SECONDS 宽限期等待自动重连，超时未恢复才退出
      4) 累计 SAMPLE_DROP_EXIT_COUNT 次「样本增量波动>5% 且丢包数增加」（持续丢包保护，优雅退出拿 log/bin）
      5) 基准期（第3/4/5周期）样本增量波动 >2%（数据不稳，优雅退出）
  - 结束时 stopDataNotification + disconnect，导出 bin，并校验 bin 文件存在
  - 支持 Ctrl+C 中断，同样走收尾流程

任何退出路径（包括连接/init/起流失败）都统一走 _teardown() 收尾，
保证 log 目录与 bin 是否存在被打印出来，便于事后定位。

前置条件：
  - 主机(电脑)：系统蓝牙已【关闭】、电源稳定（长时间运行建议插电）
  - USB dongle：已插入并绑定 WinUSB 驱动（可用 sensor/tools/setup_dongle_winusb.ps1）
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
SAMPLE_DELTA_TOLERANCE = 0.01    # 样本增量波动阈值（±1%），数据量视角的丢包交叉校验
BASELINE_CYCLES = [3, 4, 5]      # 取这几个周期的样本增量均值作为固定基准（跳过启动期前 2 个周期）
BASELINE_DISPERSION_TOLERANCE = 0.02  # 基准期（3/4/5 周期）样本增量之间的最大波动阈值（±2%），超过则数据不稳退出
SAMPLE_DROP_TOLERANCE = 0.05     # 样本增量大幅波动阈值（±5%），与丢包叠加时判为数据质量劣化
SAMPLE_DROP_EXIT_COUNT = 10      # 累计满足「波动>5% 且丢包数增加」的次数达到该值即优雅退出
RECONNECT_GRACE_SECONDS = 30     # 数据流/设备状态异常后的宽限期（秒），等待自动重连恢复，超时未恢复才退出
GRACE_POLL_INTERVAL = 5          # 宽限期内的轮询间隔（秒），尽快感知数据流恢复

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

def _dump_sample_rates(sensor):
    """打印设备当前采样率（默认），确认跑之前是否为默认采样率。

    本脚本不设置采样率，故此处读到的即为设备默认采样率。
    覆盖 DeviceInfo 的采样率字段 + EEG_SAMPLE_RATE 参数（若设备支持）。
    """
    try:
        dinfo = sensor.getDeviceInfo()
    except Exception as e:
        dinfo = None
        print(f"[采样率] getDeviceInfo 抛异常 {type(e).__name__}: {e}", flush=True)

    if dinfo is not None:
        fields = [
            "EmgSampleRate", "EegSampleRate", "EcgSampleRate",
            "AccSampleRate", "GyroSampleRate", "PpgSampleRate",
            "ImuSampleRate", "Spo2SampleRate", "ImpeSampleRate",
            "BrthSampleRate", "MagAngleSampleRate", "EulerSampleRate",
            "QuatSampleRate",
        ]
        rates = []
        for f in fields:
            try:
                v = getattr(dinfo, f)
            except Exception:
                continue
            if v is None:
                continue
            try:
                if int(v) <= 0:
                    continue
            except (TypeError, ValueError):
                pass
            rates.append(f"{f}={v}")
        if rates:
            print(f"[采样率] 设备当前采样率（默认，本脚本未设置）: {', '.join(rates)}", flush=True)
        else:
            print("[采样率] 未从 getDeviceInfo 读到有效采样率字段", flush=True)

    try:
        eeg_rate = sensor.getParam("EEG_SAMPLE_RATE")
    except Exception as e:
        eeg_rate = f"抛异常 {type(e).__name__}: {e}"
    try:
        eeg_list = sensor.getParam("EEG_SAMPLE_RATE_LIST")
    except Exception as e:
        eeg_list = f"抛异常 {type(e).__name__}: {e}"
    print(f"[采样率] getParam('EEG_SAMPLE_RATE')={eeg_rate!r}  "
          f"getParam('EEG_SAMPLE_RATE_LIST')={eeg_list!r}", flush=True)


class DataCounter:
    def __init__(self):
        self.batches = 0
        self.samples = 0
        self.calls = 0          # onDataCallback 被调用的次数（诊断：区分"没回调"还是"空回调"）
        self.lost = 0           # getLostPackageCount 累计丢包数（SDK 报告的序号 gap，待 bin 复核）
        self.lock = threading.Lock()

    def __call__(self, sensor, data):
        items = data if isinstance(data, list) else [data]
        with self.lock:
            self.calls += 1
            self.batches += len(items)
            for it in items:
                try:
                    cs = it.channelSamples
                except Exception:
                    continue
                if not cs:
                    continue
                n = 0
                try:
                    n = sum(len(ch) for ch in cs)
                except Exception:
                    try:
                        n = len(cs)
                    except Exception:
                        n = 0
                self.samples += n
                # 累加 SDK 报告的丢包数（序号 gap），仅统计 >0 的整数值
                try:
                    lp = it.getLostPackageCount()
                    if isinstance(lp, int) and lp > 0:
                        self.lost += lp
                except Exception:
                    pass

    def snapshot(self):
        with self.lock:
            return self.batches, self.samples, self.calls, self.lost

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
    # getParam 可能返回 'Error: xxx' 等错误字符串而非真实路径；
    # 只有指向真实存在的文件才可信，否则回退到目录扫描（bin 实际可能已落盘）
    bin_path = ble_path if (isinstance(ble_path, str) and ble_path.strip()) else None
    if not (bin_path and os.path.isfile(bin_path)):
        bin_path = bins_after[new_bins[0]] if new_bins else None
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


def _resolve_target_identity():
    """解析命令行参数指定的目标 identity。

    用法：python longevity.py [identity]
      - 提供参数：作为目标 identity（覆盖 config.TARGET_IDENTITY）
      - 未提供：返回 None，走 config 默认（common.TARGET_IDENTITIES）

    参数 identity 必须在 config.DEVICES 中定义，否则报错退出。
    """
    if len(sys.argv) <= 1:
        return None
    tid = sys.argv[1].strip().upper()
    if not tid:
        print("[错误] 命令行参数 identity 为空", flush=True)
        raise SystemExit(1)
    defined = {(c.get("identity") or "").strip().upper() for c in config.DEVICES}
    if tid not in defined:
        print(f"[错误] 命令行指定的 identity {tid} 未在 config.DEVICES 中定义", flush=True)
        print(f"[错误] 已定义 identity: {sorted(defined)}", flush=True)
        raise SystemExit(1)
    return tid


def main():
    target_identity = _resolve_target_identity()
    ctrl = SensorController()

    print("=" * 60, flush=True)
    print("Longevity 长稳（USB dongle 后端）：长时间起流 + bin 落盘 + debug log", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    backend = ctrl.getBLEBackendName()
    print(f"ble backend = {backend}（Windows 上可能恒为 'bleak'，仅作参考，不参与判定）", flush=True)

    # dongle 就绪检查；dongle 已可用时返回 "OK: N"，不会触发提权
    try:
        dongle_ok = checkSetupDongle()
    except Exception as e:
        dongle_ok = None
        print(f"[dongle] checkSetupDongle() 抛异常 {type(e).__name__}: {e}", flush=True)
    print(f"[dongle] checkSetupDongle() -> {dongle_ok!r}", flush=True)
    if not (isinstance(dongle_ok, str) and dongle_ok.startswith("OK")):
        print("[FAIL] USB dongle 未就绪（无可用 dongle），无法用 dongle 连接设备。", flush=True)
        ctrl.terminate()
        return

    print(f"最小运行时长: {MIN_DURATION_SECONDS}s（{MIN_DURATION_SECONDS / 3600:.1f} 小时）", flush=True)
    print(f"低电量退出阈值: {LOW_BATTERY_THRESHOLD}%", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：系统蓝牙已【关闭】、建议插电", flush=True)
    print("  - USB dongle：已插入并绑定 WinUSB 驱动", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认：1) 系统蓝牙已【关闭】；2) USB dongle 已插入；"
          "3) OYWW1100 已【开机】且在范围内。本脚本将长时间运行，完成后按回车开始 ...")

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
    if target_identity:
        print(f"\n[扫描] 目标 identity: {target_identity}（命令行指定）", flush=True)
    else:
        print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}（config 默认）", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, target_identity=target_identity)
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

    # 自动重连：异常断连后自动恢复数据流，并记录每次重连事件（终端 + SDK log）
    reconnect_lock = threading.Lock()
    reconnect_events = []

    def on_auto_reconnect(sensor, restore, *args):
        ts = time.time()
        name = getattr(getattr(sensor, 'BLEDevice', None), 'Name', '?')
        addr = getattr(getattr(sensor, 'BLEDevice', None), 'Address', '?')
        msg = (f"自动重连触发: {name} {addr} restore={restore} "
               f"@{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ts))}")
        with reconnect_lock:
            reconnect_events.append(msg)
        print(f"\n[重连] {msg}", flush=True)
        try:
            ctrl.log(f"[Longevity] {msg}", "W")
        except Exception as e:
            print(f"[重连] ctrl.log 写日志抛异常 {type(e).__name__}: {e}", flush=True)
        # 不接管恢复：让 SDK 走默认流程（重连 -> init(原参数) -> 重放 setParam -> startDataNotification）
        if args:
            try:
                args[0](False)
            except Exception:
                pass
            return None
        return False

    sensor.autoReconnect = True
    sensor.onAutoReconnect = on_auto_reconnect

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

    # 采样率信息：确认跑之前设备使用的是默认采样率（本脚本不设置采样率）
    _dump_sample_rates(sensor)

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

    # 错误回调：设备断连/异常时立即打印 + 写 SDK 错误日志（不等下一轮 60s 轮询）
    def on_error(sensor, reason):
        name = getattr(getattr(sensor, 'BLEDevice', None), 'Name', '?')
        addr = getattr(getattr(sensor, 'BLEDevice', None), 'Address', '?')
        msg = f"设备异常/断连: {name} {addr} reason={reason}"
        print(f"\n[ERROR] {msg}", flush=True)
        try:
            ctrl.log(f"[Longevity] {msg}", "E")
        except Exception as e:
            print(f"[ERROR] ctrl.log 写错误日志抛异常 {type(e).__name__}: {e}", flush=True)

    sensor.onErrorCallback = on_error

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
    baseline_samples = None  # 样本增量基准（第 3/4/5 个周期均值确定）
    prev_samples = 0         # 上一周期累计样本数
    prev_lost = 0            # 上一周期累计丢包数
    cycle_index = 0          # 周期计数（第 1/2 周期为启动期跳过，第 3/4/5 周期定基准，第 6 周期起校验）
    baseline_window = []     # 用于计算基准的样本增量（第 3/4/5 个周期）
    drop_loss_cycles = 0     # 累计「样本增量波动>5% 且丢包数增加」的周期数，达 SAMPLE_DROP_EXIT_COUNT 则退出
    drop_started_at = None   # 数据流/设备状态异常的起始时刻（进入宽限期等待自动重连）

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

            # 退出条件 3：数据流/设备状态异常（宽限期等待自动重连，超时未恢复才退出）
            transferring_ok = sensor.isDataTransfering is True
            ready_ok = sensor.deviceState == DeviceStateEx.Ready
            if not transferring_ok or not ready_ok:
                if drop_started_at is None:
                    drop_started_at = time.time()
                    drop_details = []
                    if not transferring_ok:
                        drop_details.append(f"isDataTransfering={sensor.isDataTransfering}")
                    if not ready_ok:
                        drop_details.append(f"deviceState={sensor.deviceState}")
                    print(f"[告警] 数据流/设备状态异常（{', '.join(drop_details)}），"
                          f"进入 {RECONNECT_GRACE_SECONDS}s 宽限期等待自动重连 ...", flush=True)
                    try:
                        ctrl.log(f"[Longevity] 数据流/设备状态异常（{', '.join(drop_details)}），"
                                 f"进入 {RECONNECT_GRACE_SECONDS}s 宽限期等待自动重连", "W")
                    except Exception as e:
                        print(f"[告警] ctrl.log 抛异常 {type(e).__name__}: {e}", flush=True)
                grace_elapsed = time.time() - drop_started_at
                if transferring_ok and ready_ok:
                    print(f"[恢复] 数据流已自动恢复（isDataTransfering=True, deviceState=Ready），"
                          f"断连持续 {grace_elapsed:.1f}s", flush=True)
                    try:
                        ctrl.log(f"[Longevity] 数据流已自动恢复（断连持续 {grace_elapsed:.1f}s）", "W")
                    except Exception as e:
                        print(f"[恢复] ctrl.log 抛异常 {type(e).__name__}: {e}", flush=True)
                    drop_started_at = None
                elif grace_elapsed >= RECONNECT_GRACE_SECONDS:
                    exit_reason = (f"数据流中断且 {RECONNECT_GRACE_SECONDS}s 内未自动恢复"
                                   f"（isDataTransfering={sensor.isDataTransfering}, "
                                   f"deviceState={sensor.deviceState}）")
                    break
                else:
                    # 宽限期内短轮询，尽快感知恢复
                    time.sleep(GRACE_POLL_INTERVAL)
                    continue

            batches, samples, calls, lost = counter.snapshot()

            # ---- 样本增量 ±1% 校验（第 1/2 周期跳过，第 3/4/5 周期均值定基准，第 6 周期起校验）----
            delta_samples = samples - prev_samples
            prev_samples = samples
            cycle_index += 1

            sample_ratio = None  # 本周期样本增量相对固定基准的波动比例（未定基准时为 None）
            if cycle_index in BASELINE_CYCLES:
                baseline_window.append(delta_samples)
                if cycle_index == BASELINE_CYCLES[-1]:
                    baseline_samples = sum(baseline_window) / len(baseline_window)
                    print(f"[基准] 样本增量基准 = {baseline_samples:.0f} 样本/周期"
                          f"（第{'/'.join(map(str, BASELINE_CYCLES))}个周期均值，{CHECK_INTERVAL}s/周期）", flush=True)
                    # 基准期一致性校验：第 3/4/5 周期样本增量波动不得超过 2%，否则数据不稳，优雅退出
                    if baseline_samples:
                        max_dev = max(abs(d - baseline_samples) for d in baseline_window) / baseline_samples
                        if max_dev > BASELINE_DISPERSION_TOLERANCE:
                            warn = (f"基准期样本增量波动过大: 第{'/'.join(map(str, BASELINE_CYCLES))}个周期"
                                    f"样本增量={baseline_window}，最大偏差 {max_dev:.2%} > {BASELINE_DISPERSION_TOLERANCE:.1%}")
                            print(f"[告警] {warn}", flush=True)
                            try:
                                ctrl.log(f"[Longevity] {warn}", "E")
                            except Exception as e:
                                print(f"[告警] ctrl.log 抛异常 {type(e).__name__}: {e}", flush=True)
                            exit_reason = f"基准期（第3/4/5周期）样本增量波动 {max_dev:.2%} 超过 2%，数据不稳定"
                            break
            elif cycle_index > BASELINE_CYCLES[-1] and baseline_samples:
                sample_ratio = abs(delta_samples - baseline_samples) / baseline_samples
                if sample_ratio > SAMPLE_DELTA_TOLERANCE:
                    warn = (f"样本增量异常: 本周期 {delta_samples} vs 基准 {baseline_samples:.0f}"
                            f"（波动 {sample_ratio:.2%} > {SAMPLE_DELTA_TOLERANCE:.1%}）")
                    print(f"[告警] {warn}", flush=True)
                    try:
                        ctrl.log(f"[Longevity] {warn}", "W")
                    except Exception as e:
                        print(f"[告警] ctrl.log 抛异常 {type(e).__name__}: {e}", flush=True)

            # ---- 丢包检测（SDK getLostPackageCount 累计值，新增时告警，待 bin 复核）----
            delta_lost = lost - prev_lost
            prev_lost = lost
            if delta_lost > 0:
                warn = f"检测到丢包: 本周期新增 {delta_lost} 包（累计 {lost} 包）"
                print(f"[告警] {warn}", flush=True)
                try:
                    ctrl.log(f"[Longevity] {warn}", "W")
                except Exception as e:
                    print(f"[告警] ctrl.log 抛异常 {type(e).__name__}: {e}", flush=True)

            # ---- 退出条件：样本大幅波动(>5%) 且丢包数增加，累计达阈值即优雅退出（拿 log/bin）----
            if sample_ratio is not None and sample_ratio > SAMPLE_DROP_TOLERANCE and delta_lost > 0:
                drop_loss_cycles += 1
                print(f"[告警] 样本大幅波动且丢包增加: 波动 {sample_ratio:.2%}(>{SAMPLE_DROP_TOLERANCE:.0%})、"
                      f"新增丢包 {delta_lost}，累计 {drop_loss_cycles}/{SAMPLE_DROP_EXIT_COUNT} 次", flush=True)
                try:
                    ctrl.log(f"[Longevity] 样本大幅波动且丢包增加: 波动 {sample_ratio:.2%}、"
                             f"新增丢包 {delta_lost}，累计 {drop_loss_cycles}/{SAMPLE_DROP_EXIT_COUNT} 次", "W")
                except Exception as e:
                    print(f"[告警] ctrl.log 抛异常 {type(e).__name__}: {e}", flush=True)
                if drop_loss_cycles >= SAMPLE_DROP_EXIT_COUNT:
                    exit_reason = (f"累计 {SAMPLE_DROP_EXIT_COUNT} 次样本大幅波动(>5%)且丢包数增加"
                                   f"（样本增量波动过大 + 持续丢包）")
                    break

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

            print(f"[状态] {_fmt_hms(elapsed)}  电量={power}%  数据={batches}批/{samples}样本  回调={calls}次  丢包={lost}包", flush=True)
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
    if reconnect_events:
        print(f"  自动重连: 共 {len(reconnect_events)} 次", flush=True)
        for _ev in reconnect_events:
            print(f"    - {_ev}", flush=True)
    else:
        print(f"  自动重连: 0 次", flush=True)
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
    if have_bin:
        print(f"[复核] 丢包可再用 bin_rate_check.py 独立复核 package index gap: {bin_path}", flush=True)

if __name__ == "__main__":
    main()
