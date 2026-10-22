# -*- coding: utf-8 -*-
"""MISC-FUNC-001：hasDeviceFoundCallback 查询。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-001
可自动化：auto（无需设备）

前置条件：
  - 主机(电脑)：蓝牙已开启

流程：
  1) 未注册时读 hasDeviceFoundCallback -> 断言 False
  2) 注册 onDeviceFoundCallback 后再读 -> 断言 True
  3) 清空后再读 -> 断言 False
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from common import record


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-001 hasDeviceFoundCallback 查询", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 确保初始状态：清空回调
    ctrl.onDeviceFoundCallback = None

    # 检查1：未注册时 hasDeviceFoundCallback == False
    has_cb = ctrl.hasDeviceFoundCallback
    print(f"\n[检查1] 未注册时 hasDeviceFoundCallback = {has_cb}", flush=True)
    record(results, "未注册时 hasDeviceFoundCallback==False", has_cb is False,
           "hasDeviceFoundCallback == False", f"hasDeviceFoundCallback == {has_cb}")

    # 检查2：注册后 hasDeviceFoundCallback == True
    def dummy_callback(device_list):
        pass

    ctrl.onDeviceFoundCallback = dummy_callback
    has_cb = ctrl.hasDeviceFoundCallback
    print(f"\n[检查2] 注册后 hasDeviceFoundCallback = {has_cb}", flush=True)
    record(results, "注册后 hasDeviceFoundCallback==True", has_cb is True,
           "hasDeviceFoundCallback == True", f"hasDeviceFoundCallback == {has_cb}")

    # 检查3：清空后 hasDeviceFoundCallback == False
    ctrl.onDeviceFoundCallback = None
    has_cb = ctrl.hasDeviceFoundCallback
    print(f"\n[检查3] 清空后 hasDeviceFoundCallback = {has_cb}", flush=True)
    record(results, "清空后 hasDeviceFoundCallback==False", has_cb is False,
           "hasDeviceFoundCallback == False", f"hasDeviceFoundCallback == {has_cb}")

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