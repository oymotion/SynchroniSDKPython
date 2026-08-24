# -*- coding: utf-8 -*-
"""DEV-SM-003：非目标前缀设备不误连接（防串设备）。

对应用例：02_连接与状态机.md -> DEV-SM-003
可自动化：auto（需环境中存在 ≥2 台设备，其一为目标 OB6000C，其余为"非目标"）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备（目标）：OB6000C 上电在范围内
  - 非目标设备：OB6000C 上电在范围内（用于验证"不误连"）

流程：
  1) 确认设备开机 -> 按回车
  2) scan 列出所有设备，识别目标 OB6000C 与其它（非目标）设备
  3) requireSensor(目标) + connect(目标)
  4) 断言连接集合里【只有】目标，其它扫描到的设备均未被误连
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match


def _mac_set(items):
    """从 SensorProfile 列表或 BLEDevice 列表提取大写 Address 集合。"""
    macs = set()
    for it in items:
        bd = getattr(it, 'BLEDevice', None) or it
        addr = getattr(bd, 'Address', '') or ''
        if addr:
            macs.add(addr.upper())
    return macs


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-003 非目标前缀设备不误连接（防串设备）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备（目标）：OB6000C 上电在范围内", flush=True)
    print("  - 非目标设备：OB6000C 上电在范围内（用于验证不误连）", flush=True)

    input("\n>>> [人工操作] 请确认以下设备均已【开机】且在范围内：\n"
          "    1) 目标设备 OB6000C\n"
          "    2) 非目标设备 OB6000C\n"
          "    完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    print(f"[扫描] 发现 {len(devices) if devices else 0} 台设备:", flush=True)
    for d in (devices or []):
        print(f"  - {getattr(d, 'Name', '?')} {getattr(d, 'Address', '?')}", flush=True)

    if target is None:
        print("[FAIL] 未匹配到 config 中启用的目标设备（OB6000C/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OB6000C", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    target_addr = (getattr(target, 'Address', '') or '').upper()
    target_name = getattr(target, 'Name', '?')
    print(f"[扫描] 目标设备: {target_name} {target_addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OB6000C", f"匹配到 {target_name} {target_addr}")

    # 其它（非目标）设备
    others = [d for d in (devices or [])
              if (getattr(d, 'Address', '') or '').upper() != target_addr]
    other_addrs = [(getattr(d, 'Address', '') or '').upper() for d in others]
    print(f"[识别] 非目标设备 {len(others)} 台: {other_addrs}", flush=True)

    has_others = len(others) > 0
    record(results, "环境中存在非目标设备", True if has_others else None,
           "存在 ≥1 台非目标设备", f"非目标设备 {len(others)} 台 {other_addrs}")

    # requireSensor + connect 目标
    sensor = ctrl.requireSensor(target)
    if sensor is None:
        print("[FAIL] SensorController.requireSensor 返回 None", flush=True)
        record(results, "requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "返回 None")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "requireSensor 返回 SensorProfile", isinstance(sensor, SensorProfile),
           "返回 SensorProfile", f"返回 {type(sensor).__name__}")

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

    connected = (sensor.deviceState == DeviceStateEx.Ready)

    if not connected:
        # 未连上，防串验证前提不成立，标记跳过
        record(results, "连接集合仅含目标设备（不误连）", None,
               "getConnectedSensors/Devices 仅含 OB6000C", "未连接成功，跳过防串验证")
        record(results, "非目标设备未被误连", None,
               "非目标设备不在连接集合中", "未连接成功，跳过防串验证")
    else:
        # 连接集合验证
        conn_sensors = ctrl.getConnectedSensors() or []
        conn_devices = ctrl.getConnectedDevices() or []
        sensor_macs = _mac_set(conn_sensors)
        device_macs = _mac_set(conn_devices)
        print(f"[检查] getConnectedSensors macs = {sensor_macs}", flush=True)
        print(f"[检查] getConnectedDevices macs = {device_macs}", flush=True)

        # 判定1：连接集合仅含目标（数量 1 且为目标 mac）
        only_target_sensors = (sensor_macs == {target_addr})
        record(results, "连接集合仅含目标设备（不误连）", only_target_sensors,
               f"getConnectedSensors() == {{{target_addr}}}", f"sensors={sensor_macs} devices={device_macs}")

        # 判定2：非目标设备均未出现在连接集合
        stray = [m for m in other_addrs if m in sensor_macs or m in device_macs]
        if has_others:
            record(results, "非目标设备未被误连", len(stray) == 0,
                   "非目标设备均不在连接集合中", f"被误连: {stray}" if stray else f"无（{len(others)} 台非目标均未连）")
        else:
            record(results, "非目标设备未被误连", None,
                   "非目标设备均不在连接集合中", "环境无非目标设备，无串扰对象可验证")

    # 清理
    if connected:
        try:
            sensor.disconnect()
        except Exception as e:
            print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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
    ctrl.terminate()


if __name__ == "__main__":
    main()
