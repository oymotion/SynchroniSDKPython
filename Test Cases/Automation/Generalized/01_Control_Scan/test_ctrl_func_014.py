# -*- coding: utf-8 -*-
"""CTRL-FUNC-014：onScanStateChange 回调触发（is_scanning, error_msg）。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-014
可自动化：auto（主机蓝牙开启即可，无需待测设备）

前置条件：
  - 主机(电脑)：蓝牙已开启

流程：
  1) 检查 SensorController.isEnable == True
  2) 注册 onScanStateChange（签名 (is_scanning: bool, error_msg: str)）
  3) startScan -> 断言回调 is_scanning == True
  4) stopScan -> 断言回调 is_scanning == False
  5) 断言回调参数 error_msg 为 str 类型

说明：
  SDK 1.4.4 新增 onScanStateChange / on_scan_state_change（驼峰别名），
  在扫描状态变化时通知上层（is_scanning + error_msg），与 isScanning 查询互补。
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

WAIT_TIMEOUT = 10  # 等待回调触发超时（秒）


def _wait_until(cond, timeout, interval=0.2, what=""):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(interval)
    print(f"  [等待超时] {what}（{timeout}s）", flush=True)
    return False


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-014 onScanStateChange 回调触发（is_scanning, error_msg）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)

    input("\n>>> [人工操作] 请确认【电脑】蓝牙已开启，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    events = []

    def on_scan_state(is_scanning, error_msg):
        events.append((is_scanning, error_msg))
        print(f"  [SensorController.onScanStateChange] is_scanning={is_scanning} error_msg={error_msg!r}", flush=True)

    ctrl.onScanStateChange = on_scan_state

    # startScan -> 期望回调 is_scanning == True
    print(f"\n[startScan] SensorController.startScan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        ret = ctrl.startScan(config.SCAN_TIMEOUT_MS)
        ret_txt = f"返回 {ret}"
    except Exception as e:
        ret = None
        ret_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[startScan] -> {ret_txt}", flush=True)

    got_start = _wait_until(lambda: any(v is True for v, _ in events), WAIT_TIMEOUT, 0.2,
                            "onScanStateChange(is_scanning=True)")
    print(f"[回调] startScan 后收到 {len(events)} 次回调 events={events!r}", flush=True)
    record(results, "startScan 后回调 is_scanning==True", got_start,
           "onScanStateChange 触发且 is_scanning==True", f"收到 {len(events)} 次回调 events={events!r}")

    # stopScan -> 期望回调 is_scanning == False
    print("\n[stopScan] SensorController.stopScan() ...", flush=True)
    try:
        sret = ctrl.stopScan()
        stop_txt = f"返回 {sret}"
    except Exception as e:
        sret = None
        stop_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[stopScan] -> {stop_txt}", flush=True)

    got_stop = _wait_until(lambda: any(v is False for v, _ in events), WAIT_TIMEOUT, 0.2,
                           "onScanStateChange(is_scanning=False)")
    print(f"[回调] stopScan 后收到 {len(events)} 次回调 events={events!r}", flush=True)
    record(results, "stopScan 后回调 is_scanning==False", got_stop,
           "onScanStateChange 触发且 is_scanning==False", f"收到 {len(events)} 次回调 events={events!r}")

    # 回调参数 error_msg 为 str 类型
    if events:
        all_err_str = all(isinstance(e, str) for _, e in events)
        err_vals = [e for _, e in events]
        record(results, "回调参数 error_msg 为 str", all_err_str,
               "error_msg 参数为 str 类型", f"error_msg={err_vals!r}")
    else:
        record(results, "回调参数 error_msg 为 str", None,
               "error_msg 参数为 str 类型", "未收到回调，跳过类型校验")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        elif status == "SKIP":
            print(f"  [SKIP] {name}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status == "FAIL":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
