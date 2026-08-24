# -*- coding: utf-8 -*-
"""BATT-FUNC-002：onPowerChanged 上报 0~100，±2% 滞回，不报 -1。

对应用例：07_电量日志调试.md -> BATT-FUNC-002
可自动化：auto（起流后观察电量变化，无需人工动作）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 注册 onPowerChanged 回调，记录每次上报的 level
  3) startDataNotification 起流（负载下电量变化更明显）
  4) 每 20 秒检查一次电量值，若与初始值不同则立即停止，判定 PASS
  5) 最多观察 10 分钟，若全程无变化则记录为"观察窗口内电量未变化"

说明：
  通过起流增加设备负载，提高电量变化的概率。
  电量变化即判 PASS（验证 onPowerChanged 回调能正确上报变化）。
  若 10 分钟无变化，不判 FAIL（设备可能满电或充电中），仅记录。

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

CHECK_INTERVAL = 20      # 每 20 秒检查一次电量
MAX_OBSERVE_SECONDS = 600  # 最多观察 10 分钟


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BATT-FUNC-002 onPowerChanged 上报 0~100，±2% 滞回，不报 -1", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

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

    # 注册 onPowerChanged 回调（线程安全）
    power_records = []
    lock = threading.Lock()
    changed = threading.Event()  # 电量变化时触发
    initial_level = [None]       # 用列表包装以便在闭包中修改
    latest_level = [None]

    def on_power_changed(sensor, level):
        ts = time.time()
        with lock:
            power_records.append((ts, level))
            if initial_level[0] is None:
                initial_level[0] = level
            latest_level[0] = level
            if initial_level[0] is not None and level != initial_level[0]:
                changed.set()

    sensor.onPowerChanged = on_power_changed

    # ---- 起流 ----
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
        print("[FAIL] 起流失败，设备不可用，终止测试", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # ---- 循环观察：每 20 秒检查，最多 10 分钟 ----
    print(f"\n[观察] 起流中，每 {CHECK_INTERVAL}s 检查电量变化，最多观察 {MAX_OBSERVE_SECONDS}s ...", flush=True)
    start_time = time.time()
    last_check_time = start_time
    battery_changed = False

    while time.time() - start_time < MAX_OBSERVE_SECONDS:
        elapsed = time.time() - start_time
        # 等待达到下一次检查间隔，同时监听 changed 事件
        next_check = last_check_time + CHECK_INTERVAL
        remaining = next_check - time.time()
        if remaining > 0:
            changed.wait(timeout=min(remaining, 5.0))  # 最多等 5s，避免长时间阻塞

        # 检查是否已触发变化
        if changed.is_set():
            init_lv = initial_level[0]
            cur_lv = latest_level[0]
            print(f"\n[观察] 电量变化！ {init_lv} -> {cur_lv}（耗时 {elapsed:.0f}s）", flush=True)
            battery_changed = True
            break

        # 到达检查间隔，输出当前状态
        if time.time() >= next_check:
            last_check_time = time.time()
            with lock:
                init_lv = initial_level[0]
                cur_lv = latest_level[0]
            print(f"  [检查] {elapsed:6.0f}s  初始电量={init_lv}  当前电量={cur_lv}", flush=True)

    if not battery_changed:
        elapsed = time.time() - start_time
        print(f"\n[观察] {MAX_OBSERVE_SECONDS}s 观察窗口内电量未变化（初始={initial_level[0]}，最终={latest_level[0]}）", flush=True)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    with lock:
        records_copy = list(power_records)

    print(f"\n[统计] 共收到 {len(records_copy)} 次 onPowerChanged 回调", flush=True)

    # ---- 校验 1: 电量变化 ----
    # 变化即 PASS；长时间无变化不判 FAIL（设备可能满电/充电中）
    if battery_changed:
        record(results, "onPowerChanged 上报电量变化", True,
               f"起流后电量变化（初始={initial_level[0]}，最终={latest_level[0]}）",
               f"变化耗时 {time.time() - start_time:.0f}s")
    else:
        record(results, "onPowerChanged 上报电量变化", None,
               f"起流后电量变化（初始={initial_level[0]}，最终={latest_level[0]}）",
               f"{MAX_OBSERVE_SECONDS}s 内电量未变化（设备可能满电或充电中）")

    # ---- 校验 2: 值在 0~100 ----
    if records_copy:
        all_in_range = all(0 <= lv <= 100 for _, lv in records_copy)
        no_neg_one = all(lv != -1 for _, lv in records_copy)
        levels = [lv for _, lv in records_copy]
        print(f"[校验] 上报值范围: {min(levels)}~{max(levels)}", flush=True)
        print(f"[校验] 全部在 0~100: {all_in_range}", flush=True)
        print(f"[校验] 全部不为 -1: {no_neg_one}", flush=True)
    else:
        all_in_range = False
        no_neg_one = False
        print("[校验] 未收到任何 onPowerChanged 回调", flush=True)

    record(results, "onPowerChanged 上报值在 0~100", all_in_range,
           "所有上报值在 0~100", f"共 {len(records_copy)} 次，全部在 0~100={all_in_range}")
    record(results, "onPowerChanged 不报 -1", no_neg_one,
           "所有上报值不为 -1", f"共 {len(records_copy)} 次，全部不为 -1={no_neg_one}")

    # ---- 校验 3: ±2% 滞回（无 1% 抖动）----
    if records_copy and len(records_copy) >= 2:
        levels = [lv for _, lv in records_copy]
        diff_by_one = []
        for i in range(1, len(levels)):
            if abs(levels[i] - levels[i - 1]) == 1:
                diff_by_one.append((i, levels[i - 1], levels[i]))
        hysteresis_ok = len(diff_by_one) == 0
        hysteresis_txt = f"无 1% 抖动={hysteresis_ok}"
        if diff_by_one:
            hysteresis_txt += f"（发现 {len(diff_by_one)} 处相邻差 1: {diff_by_one[:5]}）"
        print(f"[校验] ±2% 滞回（无 1% 抖动）: {hysteresis_ok}", flush=True)
    else:
        hysteresis_ok = False
        hysteresis_txt = f"回调次数不足（{len(records_copy)}），无法判断滞回"

    record(results, "onPowerChanged ±2% 滞回（无 1% 抖动）", hysteresis_ok,
           "无连续回调值相差恰好 1", hysteresis_txt)

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