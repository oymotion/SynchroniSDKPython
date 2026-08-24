# -*- coding: utf-8 -*-
"""DATA-FUNC-002：未 Ready 前 init/startDataNotification 失败。

对应用例：03_数据流.md -> DATA-FUNC-002
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

说明：
  在 Disconnected（未连接）状态下调用 init / startDataNotification，
  验证 SDK 拒绝这些数据流命令（返回非 True，或抛异常）且不崩溃。
  Connecting/Connected 阶段过渡极快，难以脚本捕获，故以 Disconnected 作为
  "未 Ready"的稳定代表状态。

  本脚本在主进程内 scan -> requireSensor 拿到未连接的 SensorProfile 后直接调用，
  不通过子进程重新 scan（避免"已连接设备不可发现"导致子进程扫不到设备）。
  若 SDK 门控失效导致 segfault，本脚本进程会直接崩溃（退出码非 0），这本身是
  明确的失败信号。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DATA-FUNC-002 未 Ready 前 init/startDataNotification 失败", flush=True)
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

    # 扫描匹配
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含启用的目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含启用的目标设备", f"匹配到 {name} {addr}")

    # requireSensor（未连接，deviceState 应 Disconnected）
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # 检查调用前状态
    state = sensor.deviceState
    print(f"\n[检查] 调用前 deviceState = {state}", flush=True)
    record(results, "命令调用前 deviceState==Disconnected", state == DeviceStateEx.Disconnected,
           "deviceState == Disconnected", f"deviceState = {state}")

    # init（Disconnected 状态）
    print(f"\n[调用] SensorProfile.init({config.PACKAGE_SAMPLE_COUNT}, {config.POWER_REFRESH_INTERVAL_MS}) ...", flush=True)
    try:
        init_ret = sensor.init(config.PACKAGE_SAMPLE_COUNT, config.POWER_REFRESH_INTERVAL_MS)
        init_txt = f"返回 {init_ret}"
    except Exception as e:
        init_ret = None
        init_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[调用] SensorProfile.init() -> {init_txt}", flush=True)
    init_rejected = (init_ret is not True)
    record(results, "SensorProfile.init 在 Ready 前被拒绝", init_rejected,
           "init 返回 False/抛异常（非 True）", f"init() -> {init_txt}")

    # startDataNotification（Disconnected 状态）
    print("\n[调用] SensorProfile.startDataNotification() ...", flush=True)
    try:
        start_ret = sensor.startDataNotification()
        start_txt = f"返回 {start_ret}"
    except Exception as e:
        start_ret = None
        start_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[调用] SensorProfile.startDataNotification() -> {start_txt}", flush=True)
    start_rejected = (start_ret is not True)
    record(results, "SensorProfile.startDataNotification 在 Ready 前被拒绝", start_rejected,
           "startDataNotification 返回 False/抛异常（非 True）",
           f"startDataNotification() -> {start_txt}")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
