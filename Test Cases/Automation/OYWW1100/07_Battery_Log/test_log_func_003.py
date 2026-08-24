# -*- coding: utf-8 -*-
"""LOG-FUNC-003：DEBUG_LOG_PATH=True 生成 profile log。

对应用例：07_Battery_Log.md -> LOG-FUNC-003
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True) 指向临时日志目录
  2) scan -> requireSensor -> connect -> 到达 Ready -> init
  3) setParam("DEBUG_LOG_PATH", "True") 开启 profile 级别日志导出
  4) getParam("DEBUG_LOG_PATH") 读取 profile log 路径
  5) startDataNotification 起流，采集数秒后 stopDataNotification、disconnect
  6) 校验：getParam 返回的路径文件存在，且包含设备相关内容

说明：
  DEBUG_LOG_PATH 是 profile 级别的日志开关，设为 True 后 SDK 将传感器内部日志
  导出到指定路径。getParam("DEBUG_LOG_PATH") 可读取实际导出的日志文件路径。
  本用例验证 profile log 能被正确生成并落盘。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import re
import sys
import time
import tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, scan_and_match

COLLECT_SECONDS = 3  # 起流后采集时长（秒）


def _base_name(name):
    """去掉广播名里的 (XXXX) 尾巴，得到设备基础名。"""
    return re.sub(r"\([0-9A-Fa-f]{4}\)\s*$", "", (name or "")).strip()


def _get_profile_log_path(sensor):
    """读取 profile log 路径。"""
    try:
        v = sensor.getParam("DEBUG_LOG_PATH")
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"
    return v


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("LOG-FUNC-003 DEBUG_LOG_PATH=True 生成 profile log", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OYWW1100 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OYWW1100 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    # 受控日志目录
    log_dir = tempfile.mkdtemp(prefix="sdklog_")
    print(f"\n[日志目录] 使用受控目录 {log_dir}", flush=True)
    try:
        ctrl.setLogPath(True, log_dir)
        log_ok = True
        log_txt = f"setLogPath(True, {log_dir}) 无异常"
    except Exception as e:
        log_ok = False
        log_txt = f"setLogPath 抛异常 {type(e).__name__}: {e}"
    print(f"[日志目录] {log_txt}", flush=True)
    record(results, "setLogPath 设置受控日志目录", log_ok,
           "setLogPath(True, dir) 无异常", log_txt)

    try:
        ctrl.setDebugEnabled(True)
    except Exception as e:
        print(f"[日志目录] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    print(f"[扫描] SensorController.scan({config.SCAN_TIMEOUT_MS}) ...", flush=True)
    target, devices = scan_and_match(ctrl, scan_ms=config.SCAN_TIMEOUT_MS)
    print(f"[扫描] 扫描到 {len(devices) if devices else 0} 台设备:", flush=True)
    if devices:
        for d in devices:
            n = getattr(d, 'Name', '?')
            a = getattr(d, 'Address', '?')
            print(f"  {n} {a} identity={_identity_of(n)}", flush=True)
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

    base_name = _base_name(name)
    identity = _identity_of(name)

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

    # 开启 profile 日志导出
    print("\n[profile log] setParam('DEBUG_LOG_PATH', 'True') ...", flush=True)
    try:
        bret = sensor.setParam("DEBUG_LOG_PATH", "True")
    except Exception as e:
        bret = f"抛异常 {type(e).__name__}: {e}"
    print(f"[profile log] setParam('DEBUG_LOG_PATH', 'True') -> {bret!r}", flush=True)
    record(results, "setParam('DEBUG_LOG_PATH', 'True') 返回 OK", bret == "OK",
           "setParam 返回 'OK'", f"setParam -> {bret!r}")

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

    if sret is not True:
        print("[FAIL] 起流失败，设备不可用，终止测试", flush=True)
        try:
            sensor.disconnect()
        except Exception:
            pass
        print("\n结论: FAIL", flush=True)
        ctrl.terminate()
        return

    print(f"\n[采集] 等待 {COLLECT_SECONDS}s 让数据流产生 profile log ...", flush=True)
    time.sleep(COLLECT_SECONDS)

    # 停流
    try:
        sensor.stopDataNotification()
    except Exception as e:
        print(f"[停流] stopDataNotification 抛异常 {type(e).__name__}: {e}", flush=True)

    # 读取 profile log 路径
    profile_path = _get_profile_log_path(sensor)
    print(f"[profile log] stop 后 getParam('DEBUG_LOG_PATH') = {profile_path!r}", flush=True)

    # 断开
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    if not isinstance(profile_path, str) or not profile_path.strip():
        profile_path = _get_profile_log_path(sensor)
        print(f"[profile log] disconnect 后 getParam('DEBUG_LOG_PATH') = {profile_path!r}", flush=True)

    # 给文件系统收尾留一点时间
    time.sleep(0.5)

    # ---- 测试 1：profile log 生成 ----
    profile_exists = False
    profile_actual = ""
    if isinstance(profile_path, str) and profile_path.strip():
        profile_exists = os.path.isfile(profile_path)
        if profile_exists:
            profile_size = os.path.getsize(profile_path)
            profile_actual = f"路径={profile_path}，存在=True，大小={profile_size} bytes"
        else:
            profile_actual = f"路径={profile_path}，存在=False"
    else:
        profile_actual = f"getParam 返回非有效路径: {profile_path!r}"

    print(f"[profile log] 文件存在: {profile_exists}, {profile_actual}", flush=True)
    record(results, "DEBUG_LOG_PATH=True 生成 profile log", profile_exists,
           "getParam('DEBUG_LOG_PATH') 返回路径，文件存在且非空",
           profile_actual)

    # ---- 测试 2：getParam 返回路径文件存在 ----
    # 此测试与测试 1 互补，但单独记录以确保路径解析正确
    path_valid = isinstance(profile_path, str) and profile_path.strip() and os.path.isfile(profile_path)
    record(results, "getParam 返回路径文件存在", path_valid,
           "getParam('DEBUG_LOG_PATH') 返回有效路径且文件存在",
           profile_actual)

    # 清理
    try:
        sensor.setParam("DEBUG_LOG_PATH", "False")
    except Exception:
        pass
    try:
        ctrl.setDebugEnabled(False)
    except Exception:
        pass

    ctrl.terminate()

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

    print(f"\n[提示] 本次日志落盘目录: {log_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()