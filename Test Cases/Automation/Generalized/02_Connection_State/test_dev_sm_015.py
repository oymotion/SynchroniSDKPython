# -*- coding: utf-8 -*-
"""DEV-SM-015：disconnect 后重新 scan 可再次发现设备。

对应用例：02_连接与状态机.md -> DEV-SM-015
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内

流程：
  1) scan 匹配目标设备 -> 记录其 MAC
  2) requireSensor -> connect 到 Ready
  3) disconnect -> 确认 deviceState==Disconnected
  4) 重新 scan（带重试，设备需时间恢复广播），按 MAC 精确匹配同一台设备
  5) 断言：断开后重新 scan 能再次发现该设备

说明：
  断开后设备应可被再次扫描发现；若 scan 找不到（例如 SDK 仍把设备当作
  “已连接/已知”而过滤掉），则影响重连与多设备管理，属缺陷。
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

DISCONNECT_TIMEOUT = 15   # disconnect 后等待迁移到 Disconnected 的超时（秒）
RE_SCAN_RETRIES = 5       # 断开后重新扫描的最大尝试次数（含首次）
RE_SCAN_INTERVAL = 2.0    # 两次重扫之间的等待秒数（设备恢复广播需要时间）


def _wait_until(cond, timeout, interval=0.5, what=""):
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
    print("DEV-SM-015 disconnect 后重新 scan 可再次发现设备", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，完成后按回车继续 ...")

    results = []

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 首次扫描匹配
    print(f"\n[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    if target is None:
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
    first_mac = addr.strip().upper()
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

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

    if ok is not True or sensor.deviceState != DeviceStateEx.Ready:
        print("[FAIL] 未到达 Ready，无法执行 disconnect 验证", flush=True)
        record(results, "connect 后到达 Ready", False, "deviceState==Ready", f"state={sensor.deviceState}")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "connect 后到达 Ready", True, "deviceState==Ready", f"state={sensor.deviceState}")

    # disconnect
    print("\n[断开] SensorProfile.disconnect() ...", flush=True)
    try:
        dret = sensor.disconnect()
        disc_txt = f"返回 {dret}"
    except Exception as e:
        dret = None
        disc_txt = f"抛异常 {type(e).__name__}: {e}"
    print(f"[断开] SensorProfile.disconnect() -> {disc_txt}", flush=True)
    record(results, "SensorProfile.disconnect 返回 True", dret is True,
           "disconnect() 返回 True", f"disconnect() -> {disc_txt}")

    # 等待异步迁移到 Disconnected
    _wait_until(lambda: sensor.deviceState == DeviceStateEx.Disconnected,
                DISCONNECT_TIMEOUT, 0.5, "deviceState==Disconnected")
    final_state = sensor.deviceState
    print(f"[检查] disconnect 后 SensorProfile.deviceState = {final_state}", flush=True)
    record(results, "disconnect 后 deviceState==Disconnected", final_state == DeviceStateEx.Disconnected,
           "deviceState == DeviceStateEx.Disconnected", f"deviceState == {final_state}")

    # ---- 重新 scan，验证同一台设备能被再次发现 ----
    print(f"\n[重扫] 断开后重新扫描，按 MAC {first_mac} 精确匹配 ...", flush=True)
    found_again = False
    found_txt = ""
    for attempt in range(1, RE_SCAN_RETRIES + 1):
        try:
            rescan_devices = ctrl.scan(config.SCAN_TIMEOUT_MS)
        except Exception as e:
            rescan_devices = None
            print(f"[重扫] 第 {attempt}/{RE_SCAN_RETRIES} 次 scan 抛异常 {type(e).__name__}: {e}", flush=True)
        if rescan_devices:
            print(f"[重扫] 第 {attempt}/{RE_SCAN_RETRIES} 次扫描到 {len(rescan_devices)} 台设备:", flush=True)
            for d in rescan_devices:
                n = getattr(d, 'Name', '?')
                a = getattr(d, 'Address', '?')
                print(f"    {n} {a}", flush=True)
        hit = None
        for d in (rescan_devices or []):
            if (getattr(d, 'Address', '') or '').upper() == first_mac:
                hit = d
                break
        if hit is not None:
            found_again = True
            found_txt = f"第 {attempt} 次重扫找到 {getattr(hit, 'Name', '?')} {first_mac}"
            break
        if attempt < RE_SCAN_RETRIES:
            print(f"[重扫] 未找到，{RE_SCAN_INTERVAL}s 后重试 ...", flush=True)
            time.sleep(RE_SCAN_INTERVAL)

    print(f"[重扫] {found_txt if found_again else f'{RE_SCAN_RETRIES} 次均未发现 MAC {first_mac}'}", flush=True)
    record(results, "disconnect 后重新 scan 可再次发现设备", found_again,
           f"断开后重新 scan 结果包含同一台设备（MAC {first_mac}）",
           found_txt if found_again else f"{RE_SCAN_RETRIES} 次重扫均未发现 MAC {first_mac}")

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
