# -*- coding: utf-8 -*-
"""CTRL-FUNC-007：requireSensor 有效/无效。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-007
可自动化：semi-auto（运行前需人工确认设备在范围内）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 人工确认待测设备已开机 -> 按回车
  2) 检查 SensorController.isEnable == True
  3) scan 发现 OB6000C -> requireSensor(有效设备) 返回 SensorProfile
  4) requireSensor(None) 返回 None
  5) requireSensor(无效对象) 返回 None
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record


def _find_ob6000c(devices):
    for d in devices or []:
        name = getattr(d, 'Name', None) or ''
        if name.upper().startswith('OB6000C'):
            return d
    return None


def _call_require(arg):
    """调用 requireSensor，返回 (返回值, 是否抛异常, 异常信息)。"""
    try:
        return SensorControllerInstance.requireSensor(arg), False, None
    except Exception as e:
        return None, True, f"{type(e).__name__}: {e}"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-007 requireSensor 有效/无效", flush=True)
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

    # 检查1：requireSensor(有效设备) 返回 SensorProfile
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    try:
        devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
    except Exception as e:
        devices = None
        print(f"[扫描] 抛异常 {type(e).__name__}: {e}", flush=True)
    target = _find_ob6000c(devices)

    if target is None:
        print("[检查1] 未发现 OB6000C 设备，无法验证有效输入", flush=True)
        record(results, "requireSensor(有效设备) 返回 SensorProfile", False,
               "requireSensor(有效 BLEDevice) 返回 SensorProfile", "未发现 OB6000C 设备")
    else:
        name = getattr(target, 'Name', '?')
        addr = getattr(target, 'Address', '?')
        print(f"[检查1] 有效设备: {name} {addr}", flush=True)
        sensor, raised, err = _call_require(target)
        if raised:
            print(f"[检查1] SensorController.requireSensor(有效) 抛异常 {err}", flush=True)
            record(results, "requireSensor(有效设备) 返回 SensorProfile", False,
                   "requireSensor(有效 BLEDevice) 返回 SensorProfile", f"抛异常 {err}")
        else:
            is_profile = isinstance(sensor, SensorProfile)
            print(f"[检查1] 返回值类型 = {type(sensor).__name__}, 是 SensorProfile = {is_profile}", flush=True)
            record(results, "requireSensor(有效设备) 返回 SensorProfile", is_profile,
                   "requireSensor(有效 BLEDevice) 返回 SensorProfile", f"返回 {type(sensor).__name__}")

    # 检查2：requireSensor(None) 返回 None
    sensor, raised, err = _call_require(None)
    if raised:
        print(f"[检查2] SensorController.requireSensor(None) 抛异常 {err}", flush=True)
        record(results, "requireSensor(None) 返回 None", False,
               "requireSensor(None) 返回 None", f"抛异常 {err}")
    else:
        print(f"[检查2] SensorController.requireSensor(None) -> {sensor!r}", flush=True)
        record(results, "requireSensor(None) 返回 None", sensor is None,
               "requireSensor(None) 返回 None", f"返回 {sensor!r}")

    # 检查3：requireSensor(无效对象) 返回 None
    invalid_obj = object()
    sensor, raised, err = _call_require(invalid_obj)
    if raised:
        print(f"[检查3] SensorController.requireSensor(无效对象) 抛异常 {err}", flush=True)
        record(results, "requireSensor(无效对象) 返回 None", False,
               "requireSensor(无效对象) 返回 None", f"抛异常 {err}")
    else:
        print(f"[检查3] SensorController.requireSensor(无效对象) -> {sensor!r}", flush=True)
        record(results, "requireSensor(无效对象) 返回 None", sensor is None,
               "requireSensor(无效对象) 返回 None", f"返回 {sensor!r}")

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
