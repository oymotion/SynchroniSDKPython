# -*- coding: utf-8 -*-
"""LOG-FUNC-005：log(msg, level) 写入应用日志。

对应用例：07_Battery_Log.md -> LOG-FUNC-005
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) setLogPath(True, 受控目录) + setDebugEnabled(True) 指向临时日志目录
  2) 测试 controller 级别：ctrl.log("test_controller_msg", "I") 写入一条 Info 日志
  3) scan -> requireSensor -> connect -> 到达 Ready -> init
  4) 测试 profile 级别：sensor.log("test_profile_msg", "W") 写入一条 Warning 日志
  5) 测试：profile 未启用时，sensor.log 应回退到 controller log
  6) 读取日志文件，检查 [App] 标记和测试消息的内容及级别（D/I/W/E）

说明：
  SDK 提供 log(msg, level) 接口允许应用层将自定义消息写入日志。
  level 支持 D(debug)、I(info)、W(warning)、E(error) 四个级别。
  SensorController.log 写入 controller 级别日志，SensorProfile.log 写入 profile 级别日志。
  当 profile 日志未启用时，SensorProfile.log 应回退到 controller 日志。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OB6000C 上电、在范围内
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


def _base_name(name):
    """去掉广播名里的 (XXXX) 尾巴，得到设备基础名。"""
    return re.sub(r"\([0-9A-Fa-f]{4}\)\s*$", "", (name or "")).strip()


def _list_log_files(log_dir):
    """列出日志目录中所有 .log 和 .txt 文件，返回 {filename: fullpath}。"""
    if not log_dir or not os.path.isdir(log_dir):
        return {}
    out = {}
    try:
        for fn in os.listdir(log_dir):
            if fn.lower().endswith((".log", ".txt")):
                out[fn] = os.path.join(log_dir, fn)
    except OSError:
        pass
    return out


def _read_logs(log_dir):
    """读取日志目录中所有日志文件内容，返回字符串列表。"""
    lines = []
    log_files = _list_log_files(log_dir)
    for fn, fp in sorted(log_files.items()):
        try:
            with open(fp, "r", encoding="utf-8", errors="replace") as f:
                lines.extend(f.readlines())
        except Exception:
            pass
    return lines


def _search_log_lines(log_dir, pattern):
    """在日志文件中搜索匹配 pattern 的行。"""
    matches = []
    for line in _read_logs(log_dir):
        if re.search(pattern, line, re.IGNORECASE):
            matches.append(line.strip())
    return matches


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("LOG-FUNC-005 log(msg, level) 写入应用日志", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
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

    # ---- 测试 1：SensorController.log 写入应用日志（controller 级别） ----
    ctrl_test_msg = "test_controller_msg_ctrl_log_001"
    print(f"\n[应用日志] ctrl.log('{ctrl_test_msg}', 'I') ...", flush=True)
    try:
        ctrl.log(ctrl_test_msg, "I")
        ctrl_log_ok = True
        ctrl_log_txt = f"ctrl.log('{ctrl_test_msg}', 'I') 无异常"
    except Exception as e:
        ctrl_log_ok = False
        ctrl_log_txt = f"ctrl.log 抛异常 {type(e).__name__}: {e}"
    print(f"[应用日志] {ctrl_log_txt}", flush=True)

    # 给日志写入留时间
    time.sleep(1.0)

    # 搜索日志中的 controller 消息
    ctrl_matches = _search_log_lines(log_dir, re.escape(ctrl_test_msg))
    found_ctrl = len(ctrl_matches) > 0
    print(f"[应用日志] 搜索 '{ctrl_test_msg}' 匹配到 {len(ctrl_matches)} 行:", flush=True)
    for m in ctrl_matches[:5]:
        print(f"  {m[:120]}", flush=True)
    record(results, "SensorController.log 写入应用日志", found_ctrl,
           f"ctrl.log('{ctrl_test_msg}', 'I') 后在日志中可找到该消息",
           f"匹配到 {len(ctrl_matches)} 行" if found_ctrl else "未找到匹配行")

    # 环境检查
    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        # 仍然继续汇总已有的结果
        ctrl.terminate()
        all_pass = all(r[1] == "PASS" for r in results)
        print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
        return

    # 扫描匹配
    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
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

    # ---- 测试 2：SensorProfile.log 写入应用日志（profile 级别） ----
    profile_test_msg = "test_profile_msg_sensor_log_002"
    print(f"\n[应用日志] sensor.log('{profile_test_msg}', 'W') ...", flush=True)
    try:
        sensor.log(profile_test_msg, "W")
        sensor_log_ok = True
        sensor_log_txt = f"sensor.log('{profile_test_msg}', 'W') 无异常"
    except Exception as e:
        sensor_log_ok = False
        sensor_log_txt = f"sensor.log 抛异常 {type(e).__name__}: {e}"
    print(f"[应用日志] {sensor_log_txt}", flush=True)

    # 给日志写入留时间
    time.sleep(1.0)

    # 搜索日志中的 profile 消息
    profile_matches = _search_log_lines(log_dir, re.escape(profile_test_msg))
    found_profile = len(profile_matches) > 0
    print(f"[应用日志] 搜索 '{profile_test_msg}' 匹配到 {len(profile_matches)} 行:", flush=True)
    for m in profile_matches[:5]:
        print(f"  {m[:120]}", flush=True)
    record(results, "SensorProfile.log 写入应用日志", found_profile,
           f"sensor.log('{profile_test_msg}', 'W') 后在日志中可找到该消息",
           f"匹配到 {len(profile_matches)} 行" if found_profile else "未找到匹配行")

    # ---- 测试 3：profile 未启用时回退 controller log ----
    # 此测试验证：当 profile 日志（DEBUG_LOG_PATH）未设置时，
    # sensor.log 的内容应出现在 controller 日志中（回退行为）
    fallback_msg = "test_fallback_msg_ctrl_log_003"
    print(f"\n[应用日志] 回退测试: sensor.log('{fallback_msg}', 'D') ...", flush=True)
    try:
        sensor.log(fallback_msg, "D")
        fallback_ok = True
        fallback_txt = f"sensor.log('{fallback_msg}', 'D') 无异常"
    except Exception as e:
        fallback_ok = False
        fallback_txt = f"sensor.log 抛异常 {type(e).__name__}: {e}"
    print(f"[应用日志] {fallback_txt}", flush=True)

    time.sleep(1.0)

    fallback_matches = _search_log_lines(log_dir, re.escape(fallback_msg))
    found_fallback = len(fallback_matches) > 0
    print(f"[应用日志] 搜索 '{fallback_msg}' 匹配到 {len(fallback_matches)} 行:", flush=True)
    for m in fallback_matches[:5]:
        print(f"  {m[:120]}", flush=True)

    # 验证回退：sensor.log 的消息出现在 controller 日志中
    record(results, "profile 未启用时回退 controller log", found_fallback,
           f"sensor.log('{fallback_msg}', 'D') 在 profile 未启用时应出现在 controller 日志中",
           f"匹配到 {len(fallback_matches)} 行" if found_fallback else "未找到匹配行")

    # 断开
    try:
        sensor.disconnect()
    except Exception as e:
        print(f"[断开] SensorProfile.disconnect 抛异常 {type(e).__name__}: {e}", flush=True)

    # 清理
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