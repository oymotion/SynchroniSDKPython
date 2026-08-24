# -*- coding: utf-8 -*-
"""CTRL-FUNC-005：onDeviceFoundCallback 周期触发。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-005
可自动化：semi-auto（运行前需人工确认设备在范围内）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) 注册 SensorController.onDeviceFoundCallback，断言 hasDeviceFoundCallback==True
  4) startScan(3000) -> 断言返回 True
  5) 等待约 2~3 个周期，断言回调周期触发（约每 3s 一次）
  6) 断言回调列表含 OB6000C 设备
  7) 连接设备后，断言已连接设备不再重复推送
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record

SCAN_PERIOD_MS = 3000  # 对应用例 startScan(3000)


def _desc(d):
    name = getattr(d, 'Name', None) or '?'
    addr = getattr(d, 'Address', None) or '?'
    return f"{name} {addr}"


def _find_ob6000c(devices):
    for d in devices or []:
        name = getattr(d, 'Name', None) or ''
        if name.upper().startswith('OB6000C'):
            return d
    return None


def _names(device_list):
    return [getattr(d, 'Name', None) or '?' for d in (device_list or [])]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-005 onDeviceFoundCallback 周期触发", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 注册回调
    found_events = []  # (相对秒数, [names])
    t0 = time.time()

    def on_found(device_list):
        names = _names(device_list)
        found_events.append((time.time() - t0, names))
        print(f"  [SensorController.onDeviceFoundCallback] t=+{time.time() - t0:.1f}s {len(names)}台 {names}", flush=True)

    ctrl.onDeviceFoundCallback = on_found

    # 检查1：注册后 hasDeviceFoundCallback == True
    has_cb = ctrl.hasDeviceFoundCallback
    print(f"[检查1] 注册后 SensorController.hasDeviceFoundCallback = {has_cb}", flush=True)
    record(results, "注册后 SensorController.hasDeviceFoundCallback==True", has_cb is True,
           "SensorController.hasDeviceFoundCallback == True", f"hasDeviceFoundCallback == {has_cb}")

    # 检查2：startScan 返回 True
    try:
        ret = ctrl.startScan(SCAN_PERIOD_MS)
        ret_txt = f"返回 {ret}"
    except Exception as e:
        ret = None
        ret_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[检查2] SensorController.startScan({SCAN_PERIOD_MS}) -> {ret_txt}", flush=True)
    record(results, "SensorController.startScan 返回 True", ret is True,
           "SensorController.startScan 返回 True", ret_txt)

    # 等待约 2~3 个周期
    time.sleep(8)
    stop_raised = False
    stop_err = None
    try:
        ctrl.stopScan()
    except Exception as e:
        stop_raised = True
        stop_err = f"{type(e).__name__}: {e}"
        print(f"[检查] SensorController.stopScan 抛异常 {stop_err}", flush=True)
    record(results, "SensorController.stopScan 不抛异常", not stop_raised,
           "SensorController.stopScan 不抛异常", f"抛异常 {stop_err}" if stop_raised else "无异常")
    time.sleep(0.5)

    # 检查3：周期触发（约每 3s 一次）
    n = len(found_events)
    intervals = [round(found_events[i][0] - found_events[i - 1][0], 2) for i in range(1, n)]
    print(f"[检查3] 回调触发 {n} 次，间隔 = {intervals}", flush=True)
    ok_period = n >= 2 and all(2.0 <= iv <= 4.0 for iv in intervals) if intervals else n >= 2
    record(results, "onDeviceFoundCallback 周期触发（约每 3s 一次）", ok_period,
           "约每 3s 触发一次（触发>=2次，间隔 2~4s）", f"触发 {n} 次，间隔={intervals}")

    # 检查4：回调列表含 OB6000C 设备
    has_ob6000c = any(any(nm.upper().startswith('OB6000C') for nm in names) for _, names in found_events)
    print(f"[检查4] 回调列表含 OB6000C 设备 = {has_ob6000c}", flush=True)
    record(results, "onDeviceFoundCallback 列表含 OB6000C 设备", has_ob6000c,
           "回调列表含 Name 以 OB6000C 开头的设备", f"含 OB6000C={has_ob6000c}")

    # 检查5：连接后已连接设备不重复推送
    print(f"\n[连接] SensorController.scan({config.SCAN_TIMEOUT_MS}) 获取设备 ...", flush=True)
    try:
        devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        devices = None
        print(f"[连接] scan 抛异常 {type(e).__name__}: {e}", flush=True)
    target = _find_ob6000c(devices)

    if target is None:
        print("[检查5] 未发现 OB6000C 设备，跳过“已连接设备不重复推送”检查", flush=True)
        record(results, "已连接设备不重复推送", False,
               "连接 OB6000C 后回调不再含该设备", "未发现 OB6000C 设备，未执行连接")
    else:
        name = getattr(target, 'Name', '?')
        sensor = ctrl.requireSensor(target)
        if sensor is None:
            print("[检查5] requireSensor 返回 None，跳过", flush=True)
            record(results, "已连接设备不重复推送", False,
                   "连接 OB6000C 后回调不再含该设备", "requireSensor 返回 None")
        else:
            try:
                ok = sensor.connect()
            except Exception as e:
                ok = False
                print(f"[连接] SensorProfile.connect 抛异常 {type(e).__name__}: {e}", flush=True)
            state = sensor.deviceState
            print(f"[连接] SensorProfile.connect() -> {ok}, deviceState = {state}", flush=True)
            if not ok:
                print("[检查5] 连接失败，跳过", flush=True)
                record(results, "已连接设备不重复推送", False,
                       "连接 OB6000C 后回调不再含该设备", f"connect 返回 {ok}")
            else:
                time.sleep(1)
                found_events.clear()
                t0 = time.time()
                ctrl.startScan(SCAN_PERIOD_MS)
                time.sleep(7)  # 至少 2 个周期，避免观察窗口过短导致假 PASS
                stop_raised = False
                stop_err = None
                try:
                    ctrl.stopScan()
                except Exception as e:
                    stop_raised = True
                    stop_err = f"{type(e).__name__}: {e}"
                    print(f"[检查] SensorController.stopScan 抛异常 {stop_err}", flush=True)
                record(results, "SensorController.stopScan 不抛异常（检查5内）", not stop_raised,
                       "SensorController.stopScan 不抛异常", f"抛异常 {stop_err}" if stop_raised else "无异常")
                still_contains = any(any(nm.upper().startswith('OB6000C') for nm in names) for _, names in found_events)
                print(f"[检查5] 连接后回调仍含 OB6000C = {still_contains}（预期 False）", flush=True)
                record(results, "已连接设备不重复推送", not still_contains,
                       "连接 OB6000C 后回调不再含该设备", f"连接后回调含 OB6000C={still_contains}")
                try:
                    sensor.disconnect()
                except Exception as e:
                    print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
