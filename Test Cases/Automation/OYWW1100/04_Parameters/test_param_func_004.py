# -*- coding: utf-8 -*-
"""PARAM-FUNC-004：EEG_SAMPLE_RATE 设置（待确认，按能力自动 SKIP）。

对应用例：04_参数.md -> PARAM-FUNC-004
可自动化：auto（能力判定后执行/跳过）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 能力判定：getParam("EEG_SAMPLE_RATE_LIST") 返回以 "Error" 开头
     → 设备未上报 EEG/ECG 采样率能力，记录"不支持，跳过"
  3) 支持时：逐个列表值 setParam("EEG_SAMPLE_RATE", 值)，校验返回 "OK"
     且 getParam("EEG_SAMPLE_RATE") 返回该值；设备同时有 EEG/ECG 时校验同写

说明：
  README：EEG_SAMPLE_RATE 值按设备上报的能力列表校验；
  无能力时 getParam("EEG_SAMPLE_RATE_LIST") 返回 "Error: Not supported"。
  腕带 OYWW1100 通常无 EEG，预期 SKIP。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import re
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, scan_and_match


def _capability_ok(sensor):
    """返回 (是否支持, 列表字符串/None)。"""
    try:
        r = sensor.getParam("EEG_SAMPLE_RATE_LIST")
    except Exception as e:
        return False, f"抛异常 {type(e).__name__}: {e}"
    if not isinstance(r, str) or r.startswith("Error"):
        return False, r
    return True, r


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-004 EEG_SAMPLE_RATE 设置", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

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
    print(f"[扫描] 目标设备: {name} {addr}", flush=True)
    record(results, "scan 匹配到目标设备", True, "scan 返回含目标设备", f"匹配到 {name} {addr}")

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
    supported, cap_txt = _capability_ok(sensor)
    print(f"\n[能力] getParam('EEG_SAMPLE_RATE_LIST') = {cap_txt!r}", flush=True)
    if not supported:
        record(results, "EEG_SAMPLE_RATE 设置生效", None,
               "支持 EEG/ECG 采样率时校验设置",
               f"getParam('EEG_SAMPLE_RATE_LIST')={cap_txt!r}，设备未上报能力")
        print("[SKIP] 设备未上报 EEG/ECG 采样率能力，跳过", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: SKIP", flush=True)
        ctrl.terminate()
        return

    rates = [x.strip() for x in cap_txt.split("|") if x.strip()]
    if not rates:
        record(results, "EEG_SAMPLE_RATE 设置生效", False,
               "能力列表非空", f"列表={cap_txt!r}")
        print("[FAIL] EEG_SAMPLE_RATE_LIST 为空", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    # 读 EEG/ECG 能力，用于同写验证
    info = sensor.getDeviceInfo()
    eeg_ch = 0
    ecg_ch = 0
    if info is not None:
        try:
            eeg_ch = int(getattr(info, 'EegChannelCount', 0) or 0)
        except Exception:
            eeg_ch = 0
        try:
            ecg_ch = int(getattr(info, 'EcgChannelCount', 0) or 0)
        except Exception:
            ecg_ch = 0
    print(f"[能力] EegChannelCount={eeg_ch} EcgChannelCount={ecg_ch}", flush=True)

    # 逐个列表值设置并核对
    for rate_val in rates:
        print(f"\n[参数] setParam('EEG_SAMPLE_RATE', {rate_val!r}) ...", flush=True)
        try:
            sret = sensor.setParam("EEG_SAMPLE_RATE", rate_val)
        except Exception as e:
            sret = f"抛异常 {type(e).__name__}: {e}"
        print(f"[参数] setParam -> {sret!r}", flush=True)

        try:
            cur = sensor.getParam("EEG_SAMPLE_RATE")
        except Exception as e:
            cur = f"抛异常 {type(e).__name__}: {e}"
        print(f"[参数] getParam('EEG_SAMPLE_RATE') = {cur!r}", flush=True)

        ok_val = (sret == "OK") and (cur == rate_val)
        record(results, f"EEG_SAMPLE_RATE 设置值 {rate_val} 生效",
               ok_val,
               f"setParam 返回 'OK' 且 getParam('EEG_SAMPLE_RATE')=='{rate_val}'",
               f"setParam->{sret!r} getParam->{cur!r}")

    # EEG/ECG 同写验证（仅当同时有 EEG 与 ECG 时）
    if eeg_ch > 0 and ecg_ch > 0:
        time.sleep(1.0)  # 等待 DeviceInfo 采样率更新
        info2 = sensor.getDeviceInfo()
        if info2 is None:
            record(results, "EEG/ECG 采样率同写为相同值", False,
                   "设置后 getDeviceInfo() 返回 DeviceInfo", "返回 None")
        else:
            try:
                eeg_rate = int(getattr(info2, 'EegSampleRate', -1))
            except Exception:
                eeg_rate = -1
            try:
                ecg_rate = int(getattr(info2, 'EcgSampleRate', -1))
            except Exception:
                ecg_rate = -1
            last_rate = rates[-1]
            try:
                last_rate_int = int(last_rate)
            except Exception:
                last_rate_int = -1
            same = (eeg_rate == last_rate_int and ecg_rate == last_rate_int)
            print(f"[同写] EegSampleRate={eeg_rate} EcgSampleRate={ecg_rate} 期望={last_rate_int}", flush=True)
            record(results, "EEG/ECG 采样率同写为相同值", same,
                   f"EegSampleRate==EcgSampleRate=={last_rate_int}",
                   f"EegSampleRate={eeg_rate} EcgSampleRate={ecg_rate}")
    else:
        record(results, "EEG/ECG 采样率同写为相同值", None,
               "同时有 EEG 与 ECG 时校验同写",
               f"EegChannelCount={eeg_ch} EcgChannelCount={ecg_ch}，无双模态，跳过")

    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

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
