# -*- coding: utf-8 -*-
"""PARAM-FUNC-006：getParam EEG_SAMPLE_RATE/LIST。

对应用例：04_参数.md -> PARAM-FUNC-006
可自动化：auto

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) getParam("EEG_SAMPLE_RATE") 返回当前采样率
  3) getParam("EEG_SAMPLE_RATE_LIST") 返回管道分隔的可选值列表
  4) 验证返回值格式合法

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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
from common import record, scan_and_match


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-006 getParam EEG_SAMPLE_RATE/LIST", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

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
        print("[FAIL] 未匹配到目标设备", flush=True)
        record(results, "scan 匹配到目标设备", False, "scan 返回含目标设备", "未匹配到目标")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    name = getattr(target, 'Name', '?')
    addr = getattr(target, 'Address', '?')
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

    # connect
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

    # 到达 Ready
    t0 = time.time()
    while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
        time.sleep(0.2)
    ready = (sensor.deviceState == DeviceStateEx.Ready)
    record(results, "connect 后到达 Ready", ready, "deviceState==Ready", f"state={sensor.deviceState}")

    if not ready:
        print("[FAIL] 未到达 Ready，无法继续", flush=True)
        try:
            sensor.disconnect()
        except Exception:
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

    # 能力判定
    info = sensor.getDeviceInfo()
    if info is not None:
        try:
            eeg_ch = int(getattr(info, 'EegChannelCount', 0) or 0)
            print(f"[能力] EegChannelCount = {eeg_ch}", flush=True)
        except Exception as e:
            eeg_ch = 0
            print(f"[能力] 读取 EegChannelCount 异常: {e}", flush=True)

    # ---- getParam("EEG_SAMPLE_RATE") ----
    print("\n[getParam] EEG_SAMPLE_RATE ...", flush=True)
    try:
        current_rate = sensor.getParam("EEG_SAMPLE_RATE")
        print(f"[getParam] EEG_SAMPLE_RATE = {current_rate!r}", flush=True)
    except Exception as e:
        current_rate = f"抛异常 {type(e).__name__}: {e}"
        print(f"[getParam] EEG_SAMPLE_RATE 异常: {e}", flush=True)

    rate_ok = (current_rate is not None and isinstance(current_rate, str) and not current_rate.startswith("Error"))
    record(results, "getParam('EEG_SAMPLE_RATE') 返回有效值",
           rate_ok,
           "getParam('EEG_SAMPLE_RATE') 返回非 Error 字符串",
           f"返回 {current_rate!r}")

    # ---- getParam("EEG_SAMPLE_RATE_LIST") ----
    print("\n[getParam] EEG_SAMPLE_RATE_LIST ...", flush=True)
    try:
        rate_list_str = sensor.getParam("EEG_SAMPLE_RATE_LIST")
        print(f"[getParam] EEG_SAMPLE_RATE_LIST = {rate_list_str!r}", flush=True)
    except Exception as e:
        rate_list_str = f"抛异常 {type(e).__name__}: {e}"
        print(f"[getParam] EEG_SAMPLE_RATE_LIST 异常: {e}", flush=True)

    # 验证格式：管道分隔的数字列表
    list_ok = False
    if isinstance(rate_list_str, str) and not rate_list_str.startswith("Error"):
        rates = [r.strip() for r in rate_list_str.split("|") if r.strip()]
        if rates:
            try:
                _ = [int(r) for r in rates]
                list_ok = True
                print(f"[getParam] EEG_SAMPLE_RATE_LIST 解析为: {rates}", flush=True)
            except ValueError:
                print(f"[getParam] EEG_SAMPLE_RATE_LIST 包含非数字值: {rates}", flush=True)
        else:
            print(f"[getParam] EEG_SAMPLE_RATE_LIST 为空列表", flush=True)
    else:
        print(f"[getParam] EEG_SAMPLE_RATE_LIST 返回 Error 或异常: {rate_list_str!r}", flush=True)

    record(results, "getParam('EEG_SAMPLE_RATE_LIST') 返回管道分隔的有效值列表",
           list_ok,
           "getParam('EEG_SAMPLE_RATE_LIST') 返回如 '250|500|1000' 的管道分隔数字列表",
           f"返回 {rate_list_str!r}")

    # 验证当前采样率在列表中
    if rate_ok and list_ok and isinstance(current_rate, str):
        supported = set(r.strip() for r in rate_list_str.split("|") if r.strip())
        in_list = current_rate in supported
        print(f"\n[验证] 当前 EEG_SAMPLE_RATE={current_rate} 是否在支持列表中: {in_list}", flush=True)
        record(results, "当前 EEG_SAMPLE_RATE 在 LIST 中",
               in_list,
               f"当前值 {current_rate} 在 {supported} 中",
               f"当前值={current_rate}, 列表={supported}")

    # 清理
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