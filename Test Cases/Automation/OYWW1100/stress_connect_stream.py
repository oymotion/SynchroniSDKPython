# -*- coding: utf-8 -*-
"""压力测试：反复连接-长时间起流-断开，尝试重现设备状态异常。

流程（循环 多 次，由 MAX_ROUNDS 控制）：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 注册 onPowerChanged 回调，记录电量
  3) startDataNotification 起流，持续 STREAM_SECONDS 秒
  4) 每 CHECK_INTERVAL 秒输出当前电量，不因电量变化提前退出
  5) stopDataNotification + disconnect
  6) 若 startDataNotification 返回 False，立即停止，记录失败轮次

与 batt_func_002 的逻辑一致：单次长时间起流观察 onPowerChanged 回调，
但循环多次，目的是复现"长时间起流后设备状态异常导致下次起流失败"。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：上电、在范围内
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
from common import _identity_of, match_target

MAX_ROUNDS = 10          # 共跑 10 轮
STREAM_SECONDS = 100     # 每轮起流时长 100 秒
CHECK_INTERVAL = 20      # 每 20 秒输出一次电量


def _on_error(sensor, reason):
    print(f"  [onError] {reason}", flush=True)


def _one_round(ctrl, round_num):
    """执行一次完整的连接-长时间起流-断开。返回 (ok, detail)。"""
    print(f"\n---- 第 {round_num}/{MAX_ROUNDS} 轮 ----", flush=True)

    # scan（最多重试 3 次，每次间隔 10 秒）
    SCAN_RETRIES = 3
    SCAN_RETRY_WAIT = 10
    target = None
    for attempt in range(1, SCAN_RETRIES + 1):
        print(f"  [scan] 目标 identity: {common.TARGET_IDENTITIES}（第 {attempt}/{SCAN_RETRIES} 次）...", flush=True)
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

        target = match_target(devices)
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

    # connect
    try:
        ok = sensor.connect()
        if not ok:
            return False, f"connect 返回 {ok}"
    except Exception as e:
        return False, f"connect 抛异常: {type(e).__name__}: {e}"

    # 到达 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        sensor.disconnect()
        return False, f"未到达 Ready（state={sensor.deviceState}）"

    # init
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        if not iret:
            sensor.disconnect()
            return False, f"init 返回 {iret}"
    except Exception as e:
        sensor.disconnect()
        return False, f"init 抛异常: {type(e).__name__}: {e}"

    # 注册 onPowerChanged 回调（线程安全）
    power_records = []
    lock = threading.Lock()
    initial_level = [None]
    latest_level = [None]

    def on_power_changed(sensor, level):
        ts = time.time()
        with lock:
            power_records.append((ts, level))
            if initial_level[0] is None:
                initial_level[0] = level
            latest_level[0] = level

    sensor.onPowerChanged = on_power_changed

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

    # ---- 长时间起流，定期输出电量 ----
    print(f"  [起流] 持续 {STREAM_SECONDS}s，每 {CHECK_INTERVAL}s 输出电量 ...", flush=True)
    start_time = time.time()
    last_check_time = start_time

    while time.time() - start_time < STREAM_SECONDS:
        elapsed = time.time() - start_time
        next_check = last_check_time + CHECK_INTERVAL
        remaining = next_check - time.time()
        if remaining > 0:
            time.sleep(min(remaining, 5.0))

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

    # 断开
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"  [断开] 抛异常: {type(e).__name__}: {e}", flush=True)

    return True, f"第 {round_num} 轮完成（{total_callbacks} 次电量回调）"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("压力测试：反复连接-长时间起流-断开", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    print(f"循环上限: {MAX_ROUNDS} 次，每次起流 {STREAM_SECONDS}s（{STREAM_SECONDS // 60} 分钟）", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，按回车开始 ...")

    if not ctrl.isEnable:
        print("[跳过] 电脑蓝牙未开启", flush=True)
        ctrl.terminate()
        return

    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass

    start_time = time.time()
    fail_round = None
    total_rounds = 0

    for i in range(1, MAX_ROUNDS + 1):
        ok, detail = _one_round(ctrl, i)
        total_rounds = i

        if ok:
            elapsed = time.time() - start_time
            print(f"  [OK] {detail}（累计 {elapsed:.0f}s）", flush=True)
        else:
            print(f"\n[FAIL] {detail}", flush=True)
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

    if fail_round is not None:
        print(f"\n  状态异常发生在第 {fail_round} 轮", flush=True)
        print(f"  详情: startDataNotification 返回非 True", flush=True)
    else:
        print(f"\n  全部 {MAX_ROUNDS} 轮通过，未复现异常", flush=True)

    ctrl.terminate()


if __name__ == "__main__":
    main()