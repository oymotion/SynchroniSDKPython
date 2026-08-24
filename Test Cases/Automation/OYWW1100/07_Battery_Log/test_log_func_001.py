# -*- coding: utf-8 -*-
"""LOG-FUNC-001：setDebugEnabled 开关 controller log。

对应用例：07_Battery_Log.md -> LOG-FUNC-001
可自动化：auto（无需设备，仅 controller 级别测试）

流程：
  1) 创建临时日志目录，setLogPath(True, temp_dir) 指向该目录
  2) setDebugEnabled(True) 开启 debug 日志
  3) 检查：日志目录中创建了日志文件（*.log 或 *.txt）
  4) setDebugEnabled(False) 关闭 debug 日志
  5) 检查：关闭后不再有新日志条目写入（或日志停止增长）

说明：
  本用例仅测试 controller 层面的 setDebugEnabled 开关能力，不涉及设备连接。
  setDebugEnabled(True) 应使 SDK 将调试日志写入 setLogPath 指定的目录；
  setDebugEnabled(False) 应停止日志输出。

前置条件：
  - 无特殊前置条件（无需设备）
"""

import os
import sys
import time
import tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record, _identity_of, match_target


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


def _get_total_size(log_files):
    """返回日志文件总大小（字节）。"""
    total = 0
    for fp in log_files.values():
        try:
            total += os.path.getsize(fp)
        except OSError:
            pass
    return total


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("LOG-FUNC-001 setDebugEnabled 开关 controller log", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 无特殊前置条件（无需设备，仅 controller 级别测试）", flush=True)

    results = []

    # 受控日志目录
    log_dir = tempfile.mkdtemp(prefix="sdklog_")
    print(f"\n[日志目录] 使用受控目录 {log_dir}", flush=True)

    # 记录设置日志路径前的文件状态
    files_before = _list_log_files(log_dir)
    print(f"[日志目录] setLogPath 前日志文件: {list(files_before.keys()) if files_before else '无'}", flush=True)

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

    if not log_ok:
        print("[FAIL] 无法设置日志目录，终止测试", flush=True)
        ctrl.terminate()
        print("\n结论: FAIL", flush=True)
        return

    # ---- 测试 1：setDebugEnabled(True) 创建日志文件 ----
    print("\n[测试] setDebugEnabled(True) ...", flush=True)
    try:
        ctrl.setDebugEnabled(True)
        debug_ok = True
        debug_txt = "setDebugEnabled(True) 无异常"
    except Exception as e:
        debug_ok = False
        debug_txt = f"setDebugEnabled(True) 抛异常 {type(e).__name__}: {e}"
    print(f"[测试] {debug_txt}", flush=True)

    # 等待日志写入
    time.sleep(2.0)

    files_after = _list_log_files(log_dir)
    new_files = sorted(set(files_after.keys()) - set(files_before.keys()))
    print(f"[日志目录] 开启后日志文件: {list(files_after.keys()) if files_after else '无'}", flush=True)
    print(f"[日志目录] 新增日志文件: {new_files if new_files else '无'}", flush=True)

    # 检查是否有日志文件
    if new_files:
        log_created = True
        log_created_txt = f"新增日志文件: {new_files}"
    elif files_after and not files_before:
        # 之前没有文件，现在有（可能全部是新文件，也可能文件名已存在）
        log_created = True
        log_created_txt = f"日志文件: {list(files_after.keys())}"
    else:
        log_created = False
        log_created_txt = f"未检测到日志文件，目录内容: {os.listdir(log_dir) if os.path.isdir(log_dir) else 'N/A'}"

    record(results, "setDebugEnabled(True) 创建日志文件", log_created,
           "setDebugEnabled(True) 后在日志目录中创建 .log 或 .txt 文件",
           log_created_txt)

    # 记录开启后的文件总大小
    size_after_enable = _get_total_size(files_after)

    # ---- 测试 2：setDebugEnabled(False) 关闭日志 ----
    print("\n[测试] setDebugEnabled(False) ...", flush=True)
    try:
        ctrl.setDebugEnabled(False)
        disable_ok = True
        disable_txt = "setDebugEnabled(False) 无异常"
    except Exception as e:
        disable_ok = False
        disable_txt = f"setDebugEnabled(False) 抛异常 {type(e).__name__}: {e}"
    print(f"[测试] {disable_txt}", flush=True)

    # 等待一段时间，确保不会再写入新日志
    time.sleep(2.0)

    files_after_disable = _list_log_files(log_dir)
    size_after_disable = _get_total_size(files_after_disable)

    # 检查：关闭后日志大小应不再增长（或增长很小）
    size_growth = size_after_disable - size_after_enable
    no_growth = size_growth <= 0
    print(f"[日志目录] 关闭前日志总大小: {size_after_enable} bytes", flush=True)
    print(f"[日志目录] 关闭后日志总大小: {size_after_disable} bytes", flush=True)
    print(f"[日志目录] 增长量: {size_growth} bytes", flush=True)

    if no_growth:
        disable_log_txt = f"日志大小未增长（关闭前={size_after_enable}，关闭后={size_after_disable}）"
    else:
        disable_log_txt = f"日志大小增长了 {size_growth} bytes（关闭前={size_after_enable}，关闭后={size_after_disable}）"

    record(results, "setDebugEnabled(False) 关闭日志", no_growth,
           "setDebugEnabled(False) 后日志文件不再增长",
           disable_log_txt)

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