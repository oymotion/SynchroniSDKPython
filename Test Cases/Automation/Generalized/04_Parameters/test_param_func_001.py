# -*- coding: utf-8 -*-
"""PARAM-FUNC-001：每个 NTF_* 键 ON/OFF 返回 OK，getParam 一致。

对应用例：04_参数.md -> PARAM-FUNC-001
可自动化：auto（设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 载入设备 spec，取 spec["streams"] 中 supported=True 且 channels>0 的流
  3) 对每个流键：
     - setParam(键,"ON"/"OFF") 返回 "OK"，且 getParam("NTF") 中对应键状态一致
       （ON 后为 ON，OFF 后为 OFF）
     - setParam 返回 "Error" 开头或抛异常：记录"无独立开关/不支持"（SKIP）

说明：
  能力来源只认 spec["streams"]（supported 且 channels>0 才算支持），
  不写死具体型号/通道数。getParam("NTF") 返回 pipe 分隔串
  "KEY|VALUE|KEY|VALUE|..."，键名与 setParam 键一致。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内
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


def _parse_pipe(s):
    """把 "KEY|VALUE|KEY|VALUE|..." 解析成 {KEY: VALUE}。"""
    out = {}
    if not s:
        return out
    parts = s.split("|")
    for i in range(0, len(parts) - 1, 2):
        out[parts[i]] = parts[i + 1]
    return out


def _load_spec(target):
    identity = common.identity_of_device(target)
    if identity is None:
        print("[FAIL] 无法从广播名提取 identity，无法载入设备 spec", flush=True)
        return None
    try:
        return common.spec_for_identity(identity)
    except Exception as e:
        print(f"[FAIL] 载入设备 spec 失败（identity={identity}）: {type(e).__name__}: {e}", flush=True)
        return None


def _supported_streams(spec):
    out = []
    for k, cap in (spec.get("streams") or {}).items():
        if isinstance(cap, dict) and cap.get("supported") is True and int(cap.get("channels") or 0) > 0:
            out.append((k, int(cap.get("channels") or 0)))
    return out


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-001 每个 NTF_* 键 ON/OFF 返回 OK，getParam 一致", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，"
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

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")

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

    # 逐个流键测试（能力来源：spec["streams"]，仅测 supported 且 channels>0 的流）
    print("\n[参数] 逐个 NTF_* 键 ON/OFF 并核对 getParam('NTF') ...", flush=True)
    for key, _channels in _supported_streams(spec):
        # setParam ON
        try:
            r_on = sensor.setParam(key, "ON")
        except Exception as e:
            r_on = f"抛异常 {type(e).__name__}: {e}"
        ntf_on = _parse_pipe(sensor.getParam("NTF") if callable(sensor.getParam) else "")
        v_on = ntf_on.get(key, "<缺失>")

        # setParam OFF
        try:
            r_off = sensor.setParam(key, "OFF")
        except Exception as e:
            r_off = f"抛异常 {type(e).__name__}: {e}"
        ntf_off = _parse_pipe(sensor.getParam("NTF") if callable(sensor.getParam) else "")
        v_off = ntf_off.get(key, "<缺失>")

        on_rejected = (not isinstance(r_on, str)) or r_on.startswith("Error") or r_on.startswith("抛异常")
        off_rejected = (not isinstance(r_off, str)) or r_off.startswith("Error") or r_off.startswith("抛异常")
        if on_rejected or off_rejected:
            print(f"[SKIP] {key}：setParam ON->{r_on!r} / OFF->{r_off!r}，无独立开关/不支持", flush=True)
            record(results, f"{key} ON/OFF 返回 OK 且 getParam 一致", None,
                   f"setParam({key}, ON/OFF) 返回 'OK' 时校验 getParam('NTF') 一致",
                   f"ON->{r_on!r}，OFF->{r_off!r}，无独立开关/不支持")
            continue

        ok_on = (r_on == "OK")
        ok_off = (r_off == "OK")
        match_on = (v_on == "ON")
        match_off = (v_off == "OFF")

        all_ok = ok_on and ok_off and match_on and match_off
        actual = (f"ON->{r_on!r}/getParam={v_on}，OFF->{r_off!r}/getParam={v_off}")
        print(f"[参数] {key}: ON->{r_on!r}(getParam={v_on})，OFF->{r_off!r}(getParam={v_off})", flush=True)
        record(results, f"{key} ON/OFF 返回 OK 且 getParam 一致", all_ok,
               f"setParam ON/OFF 均返回 'OK'，getParam('NTF') 中 {key} 为 ON/OFF", actual)

    # 清理：关闭可能被打开的流
    for key, _channels in _supported_streams(spec):
        try:
            sensor.setParam(key, "OFF")
        except Exception:
            pass

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
