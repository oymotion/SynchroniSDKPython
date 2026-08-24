# -*- coding: utf-8 -*-
"""DEV-SM-005：起流中 disconnect 自动先停流。

对应用例：02_连接与状态机.md -> DEV-SM-005
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内

流程：
  1) 确认设备开机 -> 按回车
  2) scan 匹配 OB6000C -> requireSensor -> connect 到 Ready
  3) init -> startDataNotification 起流
  4) 起流中（不显式 stopDataNotification）直接 disconnect()
  5) 断言无异常、isDataTransfering 最终 False、连接断开
"""

import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record, scan_and_match


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("DEV-SM-005 起流中 disconnect 自动先停流", flush=True)
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

    # 扫描匹配
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)

    if target is None:
        print("[FAIL] 未匹配到 config 中启用的设备（OB6000C/80F3）", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含 OB6000C", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含 OB6000C", f"匹配到 {name} {addr}")

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

    # 数据计数（观测用）
    batch_count = [0]

    def on_data(s, data):
        items = data if isinstance(data, list) else [data]
        batch_count[0] += len(items)

    sensor.onDataCallback = on_data
    sensor.onStateChanged = lambda s, st: print(f"  [SensorProfile.onStateChanged] {s.BLEDevice.Name} -> {st}", flush=True)
    sensor.onErrorCallback = lambda s, r: print(f"  [SensorProfile.onErrorCallback] {s.BLEDevice.Name}: {r!r}", flush=True)

    # connect 到 Ready
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

    # 等 Ready
    for _ in range(50):
        if sensor.deviceState == DeviceStateEx.Ready:
            break
        time.sleep(0.2)
    ready = (sensor.deviceState == DeviceStateEx.Ready)
    print(f"[检查] 连接后 SensorProfile.deviceState = {sensor.deviceState}", flush=True)
    record(results, "connect 后到达 Ready", ready, "deviceState==Ready", f"state={sensor.deviceState}")

    if not ready:
        print("[FAIL] 未到达 Ready，无法继续起流验证", flush=True)
        try:
            sensor.disconnect()
        except Exception as e:
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

    if iret is not True:
        print("[FAIL] init 失败，无法起流", flush=True)
        try:
            sensor.disconnect()
        except Exception as e:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 起流
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

    # 起流后 isDataTransfering
    transferring = sensor.isDataTransfering
    print(f"[检查] 起流后 SensorProfile.isDataTransfering = {transferring}", flush=True)
    record(results, "起流后 isDataTransfering==True", transferring is True,
           "isDataTransfering == True", f"isDataTransfering == {transferring}")

    if sret is not True:
        print("[FAIL] 起流失败，无法验证起流中断开", flush=True)
        try:
            sensor.disconnect()
        except Exception as e:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 起流中采集一小段（让断开时确实在传输中）
    print(f"\n[采集] 起流中采集 {config.COLLECT_SECONDS}s ...", flush=True)
    time.sleep(config.COLLECT_SECONDS)
    print(f"[采集] 收到 {batch_count[0]} 批数据（观测，不断言）", flush=True)

    # 起流中直接 disconnect（不显式 stopDataNotification）
    print("\n[断开] 起流中 SensorProfile.disconnect() ...", flush=True)
    disc_raised = False
    try:
        dret = sensor.disconnect()
        disc_txt = f"返回 {dret}"
    except Exception as e:
        dret = None
        disc_raised = True
        disc_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[断开] SensorProfile.disconnect() -> {disc_txt}", flush=True)
    record(results, "起流中 disconnect 不抛异常", not disc_raised,
           "disconnect() 不抛异常", f"disconnect() -> {disc_txt}")

    # 断开后 isDataTransfering
    transferring_after = sensor.isDataTransfering
    print(f"[检查] 断开后 SensorProfile.isDataTransfering = {transferring_after}", flush=True)
    record(results, "断开后 isDataTransfering==False（自动停流）", transferring_after is False,
           "isDataTransfering == False", f"isDataTransfering == {transferring_after}")

    # 断开后 deviceState
    final_state = sensor.deviceState
    print(f"[检查] 断开后 SensorProfile.deviceState = {final_state}", flush=True)
    record(results, "断开后 deviceState==Disconnected", final_state == DeviceStateEx.Disconnected,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {final_state}")

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
