# -*- coding: utf-8 -*-
"""BATT-FUNC-003：powerRefreshInterval 决定上报周期。

对应用例：07_电量日志调试.md -> BATT-FUNC-003
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready
  2) 用第一个 powerRefreshInterval（如 1000ms）init，注册 onPowerChanged，收集时间戳 20+ 秒
  3) 计算相邻回调平均间隔，校验与配置值接近（±50% 容差）
  4) disconnect，重新 connect -> Ready，用第二个 powerRefreshInterval（如 3000ms）init
  5) 重复收集与校验
  6) 记录每个 interval 的校验结果

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import sys
import time
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

# 测试用的两个 powerRefreshInterval 值（毫秒）
TEST_INTERVALS = [1000, 3000]
COLLECT_SECONDS = 20               # 每次收集回调的时长（秒）
INTERVAL_TOLERANCE = 0.5           # 容差比例（±50%）


def _connect_and_init(ctrl, target, refresh_interval):
    """连接目标设备，到达 Ready 并 init，返回 SensorProfile 或 None。"""
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] requireSensor 返回 None", flush=True)
        return None

    print("  SensorProfile.connect() ...", flush=True)
    try:
        ok = sensor.connect()
    except Exception as e:
        print(f"  connect 抛异常 {type(e).__name__}: {e}", flush=True)
        return None
    print(f"  connect -> {ok}  state={sensor.deviceState}", flush=True)
    if not ok:
        return None

    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    if sensor.deviceState != DeviceStateEx.Ready:
        print(f"  [FAIL] 未到达 Ready: state={sensor.deviceState}", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        return None

    print(f"  init({config.PACKAGE_SAMPLE_COUNT}, {refresh_interval}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, refresh_interval)
    except Exception as e:
        print(f"  init 抛异常 {type(e).__name__}: {e}", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        return None
    print(f"  init -> {iret}", flush=True)
    if iret is not True:
        try:
            sensor.disconnect()
        except Exception:
            pass
        return None

    return sensor


def _collect_callbacks(sensor, collect_seconds):
    """注册 onPowerChanged 回调，收集指定秒数，返回 [(timestamp, level), ...] 列表。"""
    records = []
    lock = threading.Lock()

    def on_power_changed(sensor, level):
        ts = time.time()
        with lock:
            records.append((ts, level))
        print(f"    [onPowerChanged] ts={ts:.3f} level={level}", flush=True)

    sensor.onPowerChanged = on_power_changed
    time.sleep(collect_seconds)

    with lock:
        return list(records)


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BATT-FUNC-003 powerRefreshInterval 决定上报周期", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

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

    # 对每个 refreshInterval 执行测试
    for idx, refresh_interval in enumerate(TEST_INTERVALS):
        print(f"\n{'─' * 40}", flush=True)
        print(f"[测试 {idx + 1}/{len(TEST_INTERVALS)}] powerRefreshInterval={refresh_interval}ms", flush=True)
        print(f"{'─' * 40}", flush=True)

        sensor = _connect_and_init(ctrl, target, refresh_interval)
        if sensor is None:
            record(results, f"powerRefreshInterval={refresh_interval}ms 连接+init", False,
                   "连接并 init 成功", "连接或 init 失败")
            continue

        record(results, f"powerRefreshInterval={refresh_interval}ms 连接+init", True,
               "连接并 init 成功", f"connect+init OK")

        print(f"  收集 onPowerChanged 回调 {COLLECT_SECONDS}s ...", flush=True)
        callback_records = _collect_callbacks(sensor, collect_seconds=COLLECT_SECONDS)

        print(f"  共收到 {len(callback_records)} 次回调", flush=True)

        # 断开
        try:
            sensor.disconnect()
        except Exception as e:
            print(f"  disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

        # 计算相邻回调间隔
        if len(callback_records) < 2:
            record(results, f"powerRefreshInterval={refresh_interval}ms 回调周期符合",
                   False,
                   f"平均间隔接近 {refresh_interval}ms（±{int(INTERVAL_TOLERANCE * 100)}%）",
                   f"回调次数不足（{len(callback_records)}），无法计算间隔")
            continue

        timestamps = [ts for ts, _ in callback_records]
        intervals = []
        for i in range(1, len(timestamps)):
            intervals.append(timestamps[i] - timestamps[i - 1])

        avg_interval = sum(intervals) / len(intervals)
        min_interval = min(intervals)
        max_interval = max(intervals)

        lower_bound = refresh_interval / 1000.0 * (1 - INTERVAL_TOLERANCE)
        upper_bound = refresh_interval / 1000.0 * (1 + INTERVAL_TOLERANCE)

        print(f"  间隔统计: 平均={avg_interval:.3f}s 最小={min_interval:.3f}s 最大={max_interval:.3f}s", flush=True)
        print(f"  期望范围: {lower_bound:.3f}s ~ {upper_bound:.3f}s", flush=True)
        print(f"  详细间隔: {[f'{v:.3f}s' for v in intervals]}", flush=True)

        interval_ok = lower_bound <= avg_interval <= upper_bound
        record(results, f"powerRefreshInterval={refresh_interval}ms 回调周期符合",
               interval_ok,
               f"平均间隔接近 {refresh_interval}ms（±{int(INTERVAL_TOLERANCE * 100)}%）",
               f"平均={avg_interval:.3f}s（期望 {refresh_interval / 1000:.3f}s）")

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

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()