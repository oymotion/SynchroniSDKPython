# -*- coding: utf-8 -*-
"""CTRL-FUNC-001：蓝牙未开启时 isEnable==False，startScan 被拒绝。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-001
可自动化：semi-auto（需人工关闭/开启蓝牙）

流程（注意：这里的"蓝牙"指【电脑】蓝牙，不是待测设备 OYWW1100）：
  1) 人工关闭电脑蓝牙 -> 按回车
     （此时待测设备 OYWW1100 保持开机，用于证明"设备在范围内但电脑蓝牙关闭仍扫不到"）
  2) 断言 isEnable == False
  3) 断言 startScan 被拒绝（未进入扫描态 isScanning==False）
  4) 断言 scan 无结果（电脑蓝牙关闭不应扫到设备）
  5) 人工重新开启电脑蓝牙 -> 按回车
     （此阶段只检查电脑蓝牙 isEnable==True，不扫描、不连接设备，
       故 OYWW1100 保持开机或关机均可，不影响结果）
  6) 断言 isEnable == True（恢复）
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-001 蓝牙未开启时 isEnable==False，startScan 被拒绝", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print(f"\n[本轮目标设备] {common.TARGET_IDENTITIES}", flush=True)

    results = []

    # ---- 阶段 1：关闭蓝牙 ----
    input("\n>>> [人工操作] 请【关闭电脑】蓝牙（不是待测设备 OYWW1100），并保持 OYWW1100 设备开机，完成后按回车继续 ...")

    is_enable = ctrl.isEnable
    print(f"\n[检查1] SensorController.isEnable = {is_enable}", flush=True)
    record(results, "蓝牙关闭时 SensorController.isEnable==False", is_enable is False,
           "SensorController.isEnable == False", f"SensorController.isEnable == {is_enable}")

    # startScan 应被拒绝（以 isScanning 未进入扫描态为准，返回值仅记录）
    try:
        ret = ctrl.startScan(config.SCAN_TIMEOUT_MS)
        ret_txt = f"返回 {ret}"
    except Exception as e:
        ret_txt = f"抛异常 {type(e).__name__}: {e}"
    is_scanning = ctrl.isScanning
    print(f"[检查2] SensorController.startScan -> {ret_txt}, SensorController.isScanning = {is_scanning}", flush=True)
    record(results, "SensorController.startScan 被拒绝且未进入扫描态", is_scanning is False,
           "SensorController.isScanning == False", f"SensorController.startScan {ret_txt}, SensorController.isScanning == {is_scanning}")

    # scan 不应扫到设备
    try:
        devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
        n = len(devices) if devices else 0
        scan_txt = f"返回 {n} 台设备"
    except Exception as e:
        n = 0
        scan_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[检查3] SensorController.scan -> {scan_txt}（此时电脑蓝牙已关闭，设备即使开机也预期扫不到）", flush=True)
    record(results, "SensorController.scan 无结果（蓝牙关闭）", n == 0, "SensorController.scan 返回 0 台设备", f"SensorController.scan {scan_txt}")

    # ---- 阶段 2：重新开启蓝牙 ----
    input("\n>>> [人工操作] 请【开启电脑】蓝牙（不是待测设备 OYWW1100），完成后按回车继续 ..."
          "\n    （本阶段只检查电脑蓝牙 isEnable，不扫描、不连接设备；设备 OYWW1100 保持开机或关机均可，不影响结果）")
    time.sleep(2)  # 等待系统刷新蓝牙使能状态
    is_enable = ctrl.isEnable
    print(f"\n[检查4] 恢复后 SensorController.isEnable = {is_enable}", flush=True)
    record(results, "恢复后 SensorController.isEnable==True", is_enable is True,
           "SensorController.isEnable == True", f"SensorController.isEnable == {is_enable}")

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
