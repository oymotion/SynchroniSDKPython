# -*- coding: utf-8 -*-
"""LOG-FUNC-004：早期 scan/connect 日志不丢失。

对应用例：07_Battery_Log.md -> LOG-FUNC-004
可自动化：auto（设备上电、在范围内为运行前置；测试中无需人工动作）

流程：
  1) 先不开启 debug 日志，执行 scan、connect、到达 Ready
  2) 连接成功后，再 setDebugEnabled(True) + setLogPath 指向临时目录
  3) 检查：日志文件中包含 scan/connect 阶段的早期事件（说明早期日志未丢失）

说明：
  SDK 内部可能缓冲早期日志，在 setDebugEnabled(True) 后才写入文件。
  本用例验证即使延迟开启日志，扫描/连接阶段的日志也不会丢失。
  通过在日志文件中搜索 scan/connect 相关的关键字来验证。

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


def _search_in_logs(log_dir, keywords):
    """在日志目录的所有日志文件中搜索关键字，返回匹配行列表。"""
    matches = []
    log_files = _list_log_files(log_dir)
    for fn, fp in sorted(log_files.items()):
        try:
            with open(fp, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    for kw in keywords:
                        if kw.lower() in line.lower():
                            matches.append(line.strip())
                            break
        except Exception:
            pass
    return matches


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("LOG-FUNC-004 早期 scan/connect 日志不丢失", flush=True)
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

    # 注意：此时尚未开启 debug 日志 — 早期日志应被 SDK 缓冲

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

    # ---- 关键步骤：在 scan/connect 完成后才开启 debug 日志 ----
    log_dir = tempfile.mkdtemp(prefix="sdklog_")
    print(f"\n[日志目录] 延迟开启日志，使用受控目录 {log_dir}", flush=True)
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
        print("[日志目录] setDebugEnabled(True) 无异常", flush=True)
    except Exception as e:
        print(f"[日志目录] setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}", flush=True)

    # 给 SDK 一些时间将缓冲的早期日志写入文件
    time.sleep(2.0)

    # ---- 检查日志中是否包含早期 scan/connect 事件 ----
    # 搜索关键字：scan, connect, BLE, 设备名, 设备地址, identity
    keywords = [
        "scan",
        "connect",
        "device",
        base_name,
        identity,
        addr.replace(":", "").upper()[-4:],
    ]
    # 去重和过滤空值
    keywords = [kw for kw in keywords if kw]
    print(f"\n[日志检查] 搜索关键字: {keywords}", flush=True)

    matches = _search_in_logs(log_dir, keywords)
    print(f"[日志检查] 匹配到 {len(matches)} 行:", flush=True)
    for m in matches[:10]:  # 最多显示 10 行
        print(f"  {m[:120]}", flush=True)

    has_early_events = len(matches) > 0
    if has_early_events:
        early_txt = f"日志中找到 {len(matches)} 条匹配早期事件的行"
    else:
        early_txt = "日志中未找到匹配早期事件的行（可能日志为空或关键字不匹配）"

    record(results, "早期 scan/connect 日志不丢失（日志含早期事件）", has_early_events,
           "延迟开启 debug 日志后，日志文件中包含 scan/connect 阶段的早期事件",
           early_txt)

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