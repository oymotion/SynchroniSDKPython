# -*- coding: utf-8 -*-
"""压力测试：反复连接-长时间起流-断开，尝试重现设备状态异常。

流程（循环 多 次，由 MAX_ROUNDS 控制）：
  0) 测试开始时长期开启日志（setLogPath + setDebugEnabled），打印日志目录
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 注册 onPowerChanged 回调，记录电量
  3) setParam("DEBUG_BLE_DATA_PATH", "True") 开启 bin 录制
  4) startDataNotification 起流，持续 STREAM_SECONDS 秒
  5) 起流期间每 CHECK_INTERVAL 秒输出电量，并监测 deviceState：
     一旦离开 Ready（复位/断连）立即记录本轮失败（针对 Cerelax 复位循环）
  6) stopDataNotification + 读取 bin 路径 + disconnect
  7) 若 startDataNotification 返回 False 或起流中途断开，立即停止并记录失败轮次
  8) 出错时打印日志目录与最近生成的 bin 文件位置，便于定位

与 batt_func_002 的逻辑一致：单次长时间起流观察 onPowerChanged 回调，
但循环多次，目的是复现"长时间起流后设备状态异常导致下次起流失败"。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：上电、在范围内
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
from common import _identity_of, match_target, resolve_target_identity

MAX_ROUNDS = 50          # 共跑 50 轮
STREAM_SECONDS = 90     # 每轮起流时长 90 秒
CHECK_INTERVAL = 30      # 每 30 秒输出一次电量
CONNECT_RETRIES = 5      # connect 重试次数（针对「连上即断」的复位循环）
CONNECT_RETRY_WAIT = 5   # connect 重试间隔（秒），给设备重启/恢复留时间
CMD_STALL_THRESHOLD = 3.0   # 命令通道停滞阈值（秒）：SDK 内部每秒轮询电量触发 onPowerChanged，超过此值未回调即命令通道异常
DATA_STALL_THRESHOLD = 3.0  # 数据流停滞阈值（秒）：超过此值未收到 onDataCallback 即数据流停止


def _on_error(sensor, reason):
    print(f"  [onError] {reason}", flush=True)


def _on_state(sensor, state):
    print(f"  [onState] {state}", flush=True)


def _on_reconnect(*args):
    print(f"  [onReconnect] {args}", flush=True)


def _list_bins(log_dir):
    """列出日志目录下所有 .bin 文件（按文件名排序）。"""
    if not log_dir or not os.path.isdir(log_dir):
        return []
    out = []
    try:
        for fn in sorted(os.listdir(log_dir)):
            if fn.lower().endswith(".bin"):
                out.append(os.path.join(log_dir, fn))
    except OSError:
        pass
    return out


def _get_ble_path(sensor):
    """读取当前 bin 录制路径（未开启/异常时返回 None）。"""
    try:
        return sensor.getParam("DEBUG_BLE_DATA_PATH")
    except Exception:
        return None


def _snapshot_files(log_dir):
    """返回 log_dir 当前 bin/log 文件集合（用于识别上一轮遗留文件）。"""
    if not log_dir or not os.path.isdir(log_dir):
        return set()
    try:
        return {fn for fn in os.listdir(log_dir)
                if fn.lower().endswith((".bin", ".txt", ".log"))}
    except OSError:
        return set()


def _cleanup_previous_round(log_dir, before_files):
    """删除上一轮遗留的 bin/log 文件，减少垃圾数据。

    在每轮成功 connect 之后调用；before_files 为 connect 之前的文件快照，
    只删除其中遗留的 bin/log，避免误删本轮刚产生、仍在写入的文件。
    SDK 长期持有的全局日志文件删除失败时忽略（保留，便于出错定位）。
    """
    removed = []
    for fn in before_files:
        p = os.path.join(log_dir, fn)
        try:
            if os.path.isfile(p):
                os.remove(p)
                removed.append(fn)
        except OSError:
            pass  # 文件被占用，保留
    if removed:
        print(f"  [清理] 已删除上一轮 {len(removed)} 个文件（bin/log）", flush=True)


def _one_round(ctrl, round_num, log_dir, target_identity=None):
    """执行一次完整的连接-长时间起流-断开。返回 (ok, detail)。"""
    print(f"\n---- 第 {round_num}/{MAX_ROUNDS} 轮 ----", flush=True)
    before_files = _snapshot_files(log_dir)

    # scan（最多重试 3 次，每次间隔 10 秒）
    SCAN_RETRIES = 3
    SCAN_RETRY_WAIT = 10
    target = None
    for attempt in range(1, SCAN_RETRIES + 1):
        if target_identity:
            print(f"  [scan] 目标 identity: {', '.join(target_identity)}（命令行指定，第 {attempt}/{SCAN_RETRIES} 次）...", flush=True)
        else:
            print(f"  [scan] 目标 identity: {', '.join(common.TARGET_IDENTITIES)}（config 默认，第 {attempt}/{SCAN_RETRIES} 次）...", flush=True)
        try:
            devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
        except Exception as e:
            return False, f"scan 抛异常: {type(e).__name__}: {e}"

        if not devices:
            print(f"  [scan] 未发现任何设备", flush=True)
        else:
            for d in devices:
                n = getattr(d, 'Name', '?')
                a = getattr(d, 'Address', '?')
                ident = _identity_of(n)
                print(f"  [scan]   设备: {n}  MAC={a}  identity={ident}", flush=True)

        target = match_target(devices, target_identity=target_identity)
        if target is not None:
            break

        if attempt < SCAN_RETRIES:
            print(f"  [scan] 未匹配到目标设备，{SCAN_RETRY_WAIT}s 后重试 ...", flush=True)
            time.sleep(SCAN_RETRY_WAIT)

    if target is None:
        return False, f"未匹配到目标设备（{SCAN_RETRIES} 次扫描均失败）"

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"  [scan] 目标: {name} {addr}", flush=True)

    # requireSensor
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        return False, "requireSensor 返回 None"

    sensor.onErrorCallback = _on_error
    sensor.onStateChanged = _on_state
    sensor.onAutoReconnect = _on_reconnect

    # connect（带重试：针对「连上即断」的复位循环，connect 失败或未到 Ready 则重试）
    connected = False
    connect_fail_reason = ""

    for attempt in range(1, CONNECT_RETRIES + 1):
        # 清理可能残留的连接状态，避免上一次半途连接影响本次
        try:
            sensor.disconnect()
        except Exception:
            pass

        try:
            ok = sensor.connect()
            connect_txt = f"返回 {ok}"
        except Exception as e:
            ok = None
            connect_txt = f"抛异常 {type(e).__name__}: {e}"

        if ok is not True:
            connect_fail_reason = f"connect {connect_txt}"
            print(f"  [连接] 第 {attempt}/{CONNECT_RETRIES} 次失败：{connect_fail_reason}", flush=True)
            if attempt < CONNECT_RETRIES:
                print(f"  [连接] {CONNECT_RETRY_WAIT}s 后重试 ...", flush=True)
                time.sleep(CONNECT_RETRY_WAIT)
            continue

        # 到达 Ready
        t0 = time.time()
        while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
            time.sleep(0.2)

        if sensor.deviceState != DeviceStateEx.Ready:
            connect_fail_reason = f"未到达 Ready（state={sensor.deviceState}）"
            print(f"  [连接] 第 {attempt}/{CONNECT_RETRIES} 次失败：{connect_fail_reason}", flush=True)
            if attempt < CONNECT_RETRIES:
                print(f"  [连接] {CONNECT_RETRY_WAIT}s 后重试 ...", flush=True)
                time.sleep(CONNECT_RETRY_WAIT)
            continue

        connected = True
        print(f"  [连接] 第 {attempt}/{CONNECT_RETRIES} 次成功，到达 Ready", flush=True)
        break

    if not connected:
        return False, f"connect 重试 {CONNECT_RETRIES} 次均失败（最后：{connect_fail_reason}）"

    # 连接成功后，清理上一轮遗留的 bin/log 文件，减少垃圾数据
    _cleanup_previous_round(log_dir, before_files)

    # init
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        if not iret:
            sensor.disconnect()
            return False, f"init 返回 {iret}"
    except Exception as e:
        sensor.disconnect()
        return False, f"init 抛异常: {type(e).__name__}: {e}"

    # 注册回调（线程安全）：onPowerChanged 作为命令通道心跳，onDataCallback 作为数据流心跳
    power_records = []
    lock = threading.Lock()
    initial_level = [None]
    latest_level = [None]
    last_power_cb_ts = [None]   # 最后一次 onPowerChanged 时间戳（命令通道心跳）
    last_data_cb_ts = [None]    # 最后一次 onDataCallback 时间戳（数据流心跳）

    def on_power_changed(sensor, level):
        ts = time.time()
        with lock:
            power_records.append((ts, level))
            last_power_cb_ts[0] = ts
            if initial_level[0] is None:
                initial_level[0] = level
            latest_level[0] = level

    sensor.onPowerChanged = on_power_changed

    def on_data(sensor, data):
        with lock:
            last_data_cb_ts[0] = time.time()

    sensor.onDataCallback = on_data

    # 开启 bin 录制（每轮生成一个 bin，供出错时定位）
    try:
        bret = sensor.setParam("DEBUG_BLE_DATA_PATH", "True")
        print(f"  [bin] setParam('DEBUG_BLE_DATA_PATH','True') -> {bret!r}", flush=True)
    except Exception as e:
        print(f"  [bin] setParam('DEBUG_BLE_DATA_PATH','True') 抛异常: {type(e).__name__}: {e}", flush=True)

    # 起流
    print(f"  [起流] ...", flush=True)
    try:
        sret = sensor.startDataNotification()
    except Exception as e:
        sret = None
        print(f"  [起流] 抛异常: {type(e).__name__}: {e}", flush=True)

    if sret is not True:
        sensor.disconnect()
        return False, f"startDataNotification 返回 {sret}（第 {round_num} 轮触发异常）"

    # ---- 长时间起流，定期输出电量 + 监测断开 ----
    print(f"  [起流] 持续 {STREAM_SECONDS}s，每 {CHECK_INTERVAL}s 输出电量，并监测断开/复位 ...", flush=True)
    start_time = time.time()
    last_check_time = start_time

    while time.time() - start_time < STREAM_SECONDS:
        # 关键：监测 deviceState，一旦离开 Ready（断开/复位），立即判定本轮失败
        st = getattr(sensor, 'deviceState', None)
        if st is not None and st != DeviceStateEx.Ready:
            msg = f"起流中途离开 Ready（state={st}，t=+{time.time() - start_time:.1f}s）——疑似复位/断连"
            print(f"  [异常] {msg}", flush=True)
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            try:
                sensor.disconnect()
            except Exception:
                pass
            return False, f"第 {round_num} 轮：{msg}"

        # 命令通道心跳监测：利用 SDK 内部每秒电量轮询的 onPowerChanged 作为命令通道心跳
        now = time.time()
        with lock:
            p_ts = last_power_cb_ts[0]
            d_ts = last_data_cb_ts[0]
        cmd_stall = (now - p_ts) if p_ts else 0.0
        data_stall = (now - d_ts) if d_ts else 0.0

        # 命令通道停滞，但数据可能还在流 → 正是「data flowing but ATT not responding」征兆
        if p_ts and cmd_stall > CMD_STALL_THRESHOLD:
            if d_ts and data_stall <= DATA_STALL_THRESHOLD:
                hmsg = (f"命令通道心跳停止（电量回调停滞 {cmd_stall:.1f}s）但数据仍在流（停滞 {data_stall:.1f}s）"
                        f"——疑似 MCU 命令处理卡死（data flowing but ATT not responding）")
            else:
                hmsg = f"命令通道心跳停止（{cmd_stall:.1f}s）且数据也已停滞（{data_stall:.1f}s）——设备完全无响应"
            print(f"  [心跳异常] t=+{now - start_time:.1f}s  {hmsg}", flush=True)
            try:
                sensor.stopDataNotification()
            except Exception:
                pass
            try:
                sensor.disconnect()
            except Exception:
                pass
            return False, f"第 {round_num} 轮：{hmsg}"

        elapsed = time.time() - start_time
        next_check = last_check_time + CHECK_INTERVAL
        remaining = next_check - time.time()
        if remaining > 0:
            time.sleep(min(remaining, 1.0))  # 缩短粒度，及时捕捉 2~3 秒的断开

        if time.time() >= next_check:
            last_check_time = time.time()
            with lock:
                init_lv = initial_level[0]
                cur_lv = latest_level[0]
            print(f"  [电量] {elapsed:6.0f}s  初始={init_lv}  当前={cur_lv}", flush=True)

    with lock:
        total_callbacks = len(power_records)
    print(f"  [电量] 本轮共收到 {total_callbacks} 次 onPowerChanged 回调", flush=True)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"  [停流] 抛异常: {type(e).__name__}: {e}", flush=True)

    # 读取 bin 路径
    ble_path = _get_ble_path(sensor)
    if ble_path:
        print(f"  [bin] 本轮 bin 路径: {ble_path}", flush=True)
    else:
        bins = _list_bins(log_dir)
        if bins:
            ble_path = bins[-1]
            print(f"  [bin] 本轮 bin 路径（目录兜底）: {ble_path}", flush=True)
        else:
            print(f"  [bin] 未读到 bin 路径（目录 {log_dir} 中暂无 .bin）", flush=True)

    # 断开
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"  [断开] 抛异常: {type(e).__name__}: {e}", flush=True)

    return True, f"第 {round_num} 轮完成（{total_callbacks} 次电量回调，bin={ble_path}）"


def main():
    ctrl = SensorControllerInstance
    target_identity = resolve_target_identity()

    print("=" * 60, flush=True)
    print("压力测试：反复连接-长时间起流-断开", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    if target_identity:
        print(f"目标 identity: {', '.join(target_identity)}（命令行指定）", flush=True)
    else:
        print(f"目标 identity: {', '.join(common.TARGET_IDENTITIES)}（config 默认）", flush=True)
    print(f"循环上限: {MAX_ROUNDS} 次，每次起流 {STREAM_SECONDS}s（{STREAM_SECONDS // 60} 分钟）", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，按回车开始 ...")

    if not ctrl.isEnable:
        print("[跳过] 电脑蓝牙未开启", flush=True)
        ctrl.terminate()
        return

    # 长期开启日志：创建日志目录并启用 debug 日志
    log_dir = tempfile.mkdtemp(prefix="sdklog_stress_")
    print(f"\n[日志] 日志目录（长期开启）: {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
    except Exception as e:
        print(f"[日志] setLogPath 异常: {type(e).__name__}: {e}", flush=True)
    try:
        ctrl.setDebugEnabled(True)
        print("[日志] setDebugEnabled(True) 已开启", flush=True)
    except Exception as e:
        print(f"[日志] setDebugEnabled 异常: {type(e).__name__}: {e}", flush=True)

    start_time = time.time()
    fail_round = None
    total_rounds = 0

    for i in range(1, MAX_ROUNDS + 1):
        ok, detail = _one_round(ctrl, i, log_dir, target_identity)
        total_rounds = i

        if ok:
            elapsed = time.time() - start_time
            print(f"  [OK] {detail}（累计 {elapsed:.0f}s）", flush=True)
        else:
            print(f"\n[FAIL] {detail}", flush=True)
            print(f"[定位] 日志目录: {log_dir}", flush=True)
            bins = _list_bins(log_dir)
            if bins:
                print(f"[定位] 已生成 bin 文件（共 {len(bins)} 个）:", flush=True)
                for b in bins:
                    print(f"    {b}", flush=True)
            else:
                print(f"[定位] 日志目录中无 .bin 文件（本轮出错点可能在起流前）", flush=True)
            fail_round = i
            break

        # 每轮之间短暂休息
        time.sleep(2)

    elapsed = time.time() - start_time

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("压力测试结果", flush=True)
    print("=" * 60, flush=True)
    print(f"  完成轮次: {total_rounds}/{MAX_ROUNDS}", flush=True)
    print(f"  总耗时: {elapsed:.0f}s（{elapsed / 60:.1f} 分钟）", flush=True)
    print(f"  日志目录: {log_dir}", flush=True)

    if fail_round is not None:
        print(f"\n  状态异常发生在第 {fail_round} 轮", flush=True)
        print(f"  详情: startDataNotification 返回非 True", flush=True)
    else:
        print(f"\n  全部 {MAX_ROUNDS} 轮通过，未复现异常", flush=True)

    ctrl.terminate()


if __name__ == "__main__":
    main()