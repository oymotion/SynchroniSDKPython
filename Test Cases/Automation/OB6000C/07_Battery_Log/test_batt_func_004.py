# -*- coding: utf-8 -*-
"""BATT-FUNC-004：起流期间 onPowerChanged 持续回调。

对应用例：07_电量日志调试.md -> BATT-FUNC-004
可自动化：auto（起流后无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 注册 onPowerChanged 回调，收集 (timestamp, level) 元组
  3) startDataNotification 起流
  4) 起流期间持续观察 COLLECT_SECONDS 秒，统计相邻回调间隔
  5) 校验：任意相邻回调间隔 ≤ 5×powerRefreshInterval
  6) 校验：直到窗口结束仍无静默（最后回调距窗口结束时间 ≤ 5×powerRefreshInterval）

说明：
  powerRefreshInterval 使用配置值（默认 1000ms），COLLECT_SECONDS 默认 120s；
  规范建议 ≥10 min，但自动化测试折中为 120s 以便实用，可通过修改 COLLECT_SECONDS 调整。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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

COLLECT_SECONDS = 120  # 起流后观察时长（秒），规范建议 ≥10 min，这里折中默认 120s


def main():
    ctrl = SensorControllerInstance

    refresh_ms = config.POWER_REFRESH_INTERVAL_MS
    max_interval_s = 5 * refresh_ms / 1000.0  # 5×powerRefreshInterval（秒）

    print("=" * 60, flush=True)
    print("BATT-FUNC-004 起流期间 onPowerChanged 持续回调", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"powerRefreshInterval = {refresh_ms}ms", flush=True)
    print(f"最大允许间隔 = {max_interval_s:.1f}s（5×{refresh_ms}ms）", flush=True)
    print(f"观察时长 = {COLLECT_SECONDS}s", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
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

    # init（使用较小的 powerRefreshInterval 以便短时间内观察足够多次回调）
    print(f"\n[init] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {refresh_ms}) ...", flush=True)
    try:
        iret = sensor.init(config.PACKAGE_SAMPLE_COUNT, refresh_ms)
        init_txt = f"返回 {iret}"
    except Exception as e:
        iret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[init] SensorProfile.init() -> {init_txt}", flush=True)
    record(results, "SensorProfile.init 返回 True", iret is True, "init() 返回 True", f"init() -> {init_txt}")

    if iret is not True:
        print("[FAIL] init 失败，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 注册 onPowerChanged 回调
    power_records = []
    lock = threading.Lock()

    def on_power_changed(sensor, level):
        ts = time.time()
        with lock:
            power_records.append((ts, level))
        print(f"  [onPowerChanged] ts={ts:.3f} level={level}", flush=True)

    sensor.onPowerChanged = on_power_changed
    print("\n[观察] 已注册 onPowerChanged 回调", flush=True)

    # 起流
    collect_start = time.time()
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

    if sret is not True:
        print("[FAIL] startDataNotification 失败，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 起流期间持续观察
    print(f"\n[观察] 起流期间等待 {COLLECT_SECONDS}s，观察 onPowerChanged 回调 ...", flush=True)
    time.sleep(COLLECT_SECONDS)

    collect_end = time.time()

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    with lock:
        records_copy = list(power_records)

    print(f"\n[观察] 共收到 {len(records_copy)} 次 onPowerChanged 回调", flush=True)

    # 校验：相邻回调间隔 ≤ 5×powerRefreshInterval
    if len(records_copy) < 2:
        record(results, "起流期间 onPowerChanged 持续回调（间隔 ≤ 5×刷新周期）", False,
               f"任意相邻回调间隔 ≤ {max_interval_s:.1f}s", f"回调次数不足（{len(records_copy)}），无法计算间隔")
    else:
        timestamps = [ts for ts, _ in records_copy]
        intervals = []
        for i in range(1, len(timestamps)):
            intervals.append(timestamps[i] - timestamps[i - 1])

        max_observed = max(intervals)
        all_within = all(iv <= max_interval_s for iv in intervals)

        print(f"[校验] 相邻回调间隔: 最小={min(intervals):.3f}s 最大={max_observed:.3f}s 平均={sum(intervals)/len(intervals):.3f}s", flush=True)
        print(f"[校验] 最大允许间隔: {max_interval_s:.1f}s", flush=True)
        print(f"[校验] 全部间隔 ≤ 最大允许: {all_within}", flush=True)

        # 额外校验：最后回调距窗口结束时间 ≤ 5×powerRefreshInterval（无静默）
        last_callback_ts = timestamps[-1]
        gap_to_end = collect_end - last_callback_ts
        no_silence = gap_to_end <= max_interval_s
        print(f"[校验] 最后回调距窗口结束: {gap_to_end:.3f}s（允许 ≤ {max_interval_s:.1f}s）", flush=True)
        print(f"[校验] 窗口结束前无静默: {no_silence}", flush=True)

        overall_ok = all_within and no_silence
        result_txt = (
            f"共 {len(records_copy)} 次回调，最大间隔={max_observed:.3f}s "
            f"（允许 ≤ {max_interval_s:.1f}s），"
            f"末尾间隔={gap_to_end:.3f}s"
        )

        record(results, "起流期间 onPowerChanged 持续回调（间隔 ≤ 5×刷新周期）", overall_ok,
               f"任意相邻回调间隔 ≤ {max_interval_s:.1f}s，且窗口结束前无静默",
               result_txt)

    # 断开
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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