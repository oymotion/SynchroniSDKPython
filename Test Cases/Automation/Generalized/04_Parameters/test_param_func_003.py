# -*- coding: utf-8 -*-
"""PARAM-FUNC-003：NTF_IMU 联动四个 NTF_GFORCE_*（按设备 spec 判定适用性）。

对应用例：04_参数.md -> PARAM-FUNC-003
可自动化：auto（能力判定后执行/跳过；设备上电、在范围内为运行前置）

流程：
  1) scan -> requireSensor -> connect -> 到达 Ready -> init
  2) 载入设备 spec，读 spec["streams"] 判定四个 NTF_GFORCE_* 是否有独立开关
  3) 四个 NTF_GFORCE_* 均为 supported=False（或不存在）→ 无独立开关，
     双向联动不适用，记录"不适用，跳过"（SKIP）
  4) 存在 supported=True 的独立开关 → NTF_IMU ON 后核对相关 NTF_GFORCE_*
     为 ON，OFF 后核对为 OFF

说明：
  能力来源只认 spec["streams"]，不写死具体型号；聚合流打包（无独立开关）
  的设备本用例恒为"不适用"。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：已上电、在范围内
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


def _parse_pipe(s):
    """把 "KEY|VALUE|KEY|VALUE|..." 解析成 {KEY: VALUE}。"""
    out = {}
    if not s:
        return out
    parts = s.split("|")
    for i in range(0, len(parts) - 1, 2):
        out[parts[i]] = parts[i + 1]
    return out


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-003 NTF_IMU 联动四个 NTF_GFORCE_*", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：已上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备已【开机】且在范围内，"
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

    spec = _load_spec(target)
    if spec is None:
        record(results, "载入设备 spec", False, "能从广播名提取 identity 并载入对应 spec", "spec 载入失败")
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return
    record(results, "载入设备 spec", True, "能从广播名提取 identity 并载入对应 spec", f"model={spec.get('model')}")

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

    # 能力判定（来源：spec["streams"]）
    gforce_keys = ["NTF_GFORCE_ACC", "NTF_GFORCE_GYRO", "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT"]
    streams = spec.get("streams") or {}
    indep_keys = [k for k in gforce_keys
                  if streams.get(k) and streams[k].get("supported") is True and (streams[k].get("channels") or 0) > 0]

    if not indep_keys:
        record(results, "NTF_IMU 与四个 NTF_GFORCE_* 双向联动", None,
               "存在 NTF_GFORCE_* 独立开关时校验双向联动",
               "spec 标记 NTF_GFORCE_* 均无独立开关，双向联动不适用")
        print("[SKIP] spec 标记 NTF_GFORCE_* 均无独立开关，双向联动不适用，跳过", flush=True)
    else:
        print("\n[联动] NTF_IMU ON/OFF 联动 NTF_GFORCE_* ...", flush=True)
        try:
            r_on = sensor.setParam("NTF_IMU", "ON")
        except Exception as e:
            r_on = f"抛异常 {type(e).__name__}: {e}"
        ntf_on = _parse_pipe(sensor.getParam("NTF") if callable(sensor.getParam) else "")
        try:
            r_off = sensor.setParam("NTF_IMU", "OFF")
        except Exception as e:
            r_off = f"抛异常 {type(e).__name__}: {e}"
        ntf_off = _parse_pipe(sensor.getParam("NTF") if callable(sensor.getParam) else "")

        states_on = {k: ntf_on.get(k, "<缺失>") for k in indep_keys}
        states_off = {k: ntf_off.get(k, "<缺失>") for k in indep_keys}
        ok_on = (r_on == "OK") and all(v == "ON" for v in states_on.values())
        ok_off = (r_off == "OK") and all(v == "OFF" for v in states_off.values())
        all_ok = ok_on and ok_off
        actual = (f"NTF_IMU ON->{r_on!r}/{states_on}，OFF->{r_off!r}/{states_off}")
        print(f"[联动] {actual}", flush=True)
        record(results, "NTF_IMU 与四个 NTF_GFORCE_* 双向联动", all_ok,
               "NTF_IMU ON 后 NTF_GFORCE_* 为 ON，OFF 后为 OFF", actual)

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
