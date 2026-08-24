# -*- coding: utf-8 -*-
"""CTRL-FUNC-006：stopScan 后回调停止。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-006
可自动化：semi-auto（运行前需人工确认设备在范围内）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) 注册 onDeviceFoundCallback，startScan(3000) -> 断言返回 True
  4) 等待约 1 个周期，断言回调在触发（>=1 次）
  5) stopScan -> 断言 isScanning == False
  6) 再等待约 2 个周期，断言无新回调
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


def _names(device_list):
    return [getattr(d, 'Name', None) or '?' for d in (device_list or [])]


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-006 stopScan 后回调停止", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 注册回调
    found_events = []
    t0 = time.time()

    def on_found(device_list):
        found_events.append((time.time() - t0, _names(device_list)))
        print(f"  [SensorController.onDeviceFoundCallback] t=+{time.time() - t0:.1f}s {len(_names(device_list))}台", flush=True)

    ctrl.onDeviceFoundCallback = on_found

    # 检查1：startScan 返回 True
    try:
        ret = ctrl.startScan(SCAN_PERIOD_MS)
        ret_txt = f"返回 {ret}"
    except Exception as e:
        ret = None
        ret_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[检查1] SensorController.startScan({SCAN_PERIOD_MS}) -> {ret_txt}", flush=True)
    record(results, "SensorController.startScan 返回 True", ret is True,
           "SensorController.startScan 返回 True", ret_txt)

    # 检查2：停止前回调在触发（轮询等待，避免固定 sleep 错过首次回调）
    wait_t0 = time.time()
    while len(found_events) < 1 and time.time() - wait_t0 < 15:
        time.sleep(0.2)
    n_before = len(found_events)
    print(f"[检查2] 停止前回调触发 {n_before} 次（已等待 {time.time() - wait_t0:.1f}s）", flush=True)
    record(results, "stopScan 前 onDeviceFoundCallback 在触发", n_before >= 1,
           "回调触发 >= 1 次", f"触发 {n_before} 次")

    # stopScan（异常应计为 FAIL，而非仅警告）
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

    # 检查3：stopScan 后 isScanning == False
    is_scanning = ctrl.isScanning
    print(f"[检查3] stopScan 后 SensorController.isScanning = {is_scanning}", flush=True)
    record(results, "stopScan 后 SensorController.isScanning==False", is_scanning is False,
           "SensorController.isScanning == False", f"SensorController.isScanning == {is_scanning}")

    # 检查4：stopScan 后无新回调（先排空在途回调，再判稳定期不增长）
    DRAIN_SECONDS = SCAN_PERIOD_MS / 1000.0       # 排空窗口：1 个周期
    STABLE_SECONDS = 2 * SCAN_PERIOD_MS / 1000.0  # 稳定窗口：2 个周期

    time.sleep(DRAIN_SECONDS)  # 排空 stopScan 前在途的最后一个扫描周期回调
    n_baseline = len(found_events)
    time.sleep(STABLE_SECONDS)
    n_after = len(found_events)
    growth = n_after - n_baseline
    print(f"[检查4] stopScan 后：排空后 {n_baseline} 次 -> 稳定期后 {n_after} 次（新增 {growth}）", flush=True)
    record(results, "stopScan 后无新回调", growth == 0,
           "排空 1 周期后，稳定期(2 周期)内回调次数不再增长",
           f"排空后 {n_baseline} 次，稳定期后 {n_after} 次（新增 {growth}）")

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
