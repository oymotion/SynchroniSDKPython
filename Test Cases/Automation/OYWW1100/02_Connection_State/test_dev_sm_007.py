# -*- coding: utf-8 -*-
"""DEV-SM-007：BLEDevice 的 Name/Address/RSSI 与扫描一致。

对应用例：02_连接与状态机.md -> DEV-SM-007
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OYWW1100，记录扫描结果的 Name/Address/RSSI
  3) requireSensor -> 读 sensor.BLEDevice 的 Name/Address/RSSI
  4) 断言三者与扫描结果一致
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
    print("DEV-SM-007 BLEDevice 的 Name/Address/RSSI 与扫描一致", flush=True)
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
        print("[FAIL] 未匹配到 config 中启用的设备（OYWW1100/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OYWW1100", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 扫描结果的元数据
    scan_name = getattr(target, 'Name', '')
    scan_addr = (getattr(target, 'Address', '') or '').upper()
    scan_rssi = getattr(target, 'RSSI', None)
    print(f"[扫描] 目标: Name={scan_name!r} Address={scan_addr!r} RSSI={scan_rssi!r}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OYWW1100", f"匹配到 {scan_name} {scan_addr}")

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

    # sensor.BLEDevice 的元数据
    bd = sensor.BLEDevice
    bd_name = getattr(bd, 'Name', '')
    bd_addr = (getattr(bd, 'Address', '') or '').upper()
    bd_rssi = getattr(bd, 'RSSI', None)
    print(f"[sensor.BLEDevice] Name={bd_name!r} Address={bd_addr!r} RSSI={bd_rssi!r}", flush=True)

    # 比对 Name
    name_ok = (bd_name == scan_name)
    record(results, "SensorProfile.BLEDevice.Name 一致", name_ok,
           f"Name == {scan_name!r}", f"Name == {bd_name!r}")

    # 比对 Address
    addr_ok = (bd_addr == scan_addr)
    record(results, "SensorProfile.BLEDevice.Address 一致", addr_ok,
           f"Address == {scan_addr!r}", f"Address == {bd_addr!r}")

    # 比对 RSSI
    if bd_rssi is None:
        rssi_ok = False
        rssi_actual = "RSSI 为 None（未保留）"
    elif scan_rssi is None:
        rssi_ok = False
        rssi_actual = f"扫描 RSSI 为 None，sensor={bd_rssi!r}"
    else:
        rssi_ok = (bd_rssi == scan_rssi)
        rssi_actual = f"sensor.RSSI={bd_rssi!r} vs scan.RSSI={scan_rssi!r}"
    record(results, "SensorProfile.BLEDevice.RSSI 一致", rssi_ok,
           f"RSSI == {scan_rssi!r}", rssi_actual)

    # 清理（本用例未连接，无需 disconnect，但确保状态干净）
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
