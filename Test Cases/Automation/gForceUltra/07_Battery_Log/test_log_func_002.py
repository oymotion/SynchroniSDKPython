# -*- coding: utf-8 -*-
"""LOG-FUNC-002：setLogPath 目录/已存在文件。

对应用例：07_Battery_Log.md -> LOG-FUNC-002
可自动化：auto（无需设备，仅 controller 级别测试）

流程：
  1) setLogPath(True, new_dir) — 目录不存在，应自动创建
  2) 检查：目录已创建且可写入
  3) setLogPath(True, existing_file_path) — 指向一个已存在的文件，应被拒绝或报错
  4) 检查：setLogPath 返回错误或抛异常

说明：
  本用例测试 setLogPath 的边界行为：
  - 传入不存在的目录路径时，SDK 应自动创建目录
  - 传入一个已存在的文件路径（而非目录）时，SDK 应拒绝该操作

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


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("LOG-FUNC-002 setLogPath 目录/已存在文件", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 无特殊前置条件（无需设备，仅 controller 级别测试）", flush=True)

    results = []

    # ---- 测试 1：setLogPath 指向不存在的目录，应自动创建 ----
    # 使用 tempfile.mkdtemp 创建父目录，在其下指定一个不存在的子目录
    parent_dir = tempfile.mkdtemp(prefix="sdklog_parent_")
    new_dir = os.path.join(parent_dir, "should_be_created")
    print(f"\n[测试1] setLogPath(True, '{new_dir}')", flush=True)
    print(f"[测试1] 参数: enable=True, path='{new_dir}'", flush=True)
    print(f"[测试1] 目录是否存在（操作前）: {os.path.isdir(new_dir)}", flush=True)
    print(f"[测试1] 父目录是否存在: {os.path.isdir(parent_dir)}", flush=True)

    dir_created = False
    dir_created_txt = ""
    try:
        ret = ctrl.setLogPath(True, new_dir)
        print(f"[测试1] setLogPath 返回值: {ret}", flush=True)
        time.sleep(0.3)
        if os.path.isdir(new_dir):
            # 进一步检查目录中是否生成了日志文件
            content = os.listdir(new_dir) if os.path.isdir(new_dir) else []
            dir_created = True
            dir_created_txt = f"目录已自动创建，内含 {len(content)} 个文件/目录: {content}"
        else:
            dir_created = False
            dir_created_txt = f"setLogPath 返回 {ret}，但目录未创建: {new_dir}（父目录内容: {os.listdir(parent_dir)}）"
    except Exception as e:
        dir_created = False
        dir_created_txt = f"setLogPath 抛异常 {type(e).__name__}: {e}"

    print(f"[测试1] {dir_created_txt}", flush=True)
    record(results, "setLogPath 创建目录成功", dir_created,
           "setLogPath(True, 不存在的目录) 自动创建目录",
           dir_created_txt)

    # ---- 测试 2：setLogPath 指向已存在的文件，应被拒绝 ----
    # 创建一个临时文件
    fd, temp_file = tempfile.mkstemp(suffix=".txt", prefix="existing_file_")
    os.close(fd)
    print(f"\n[测试2] setLogPath(True, '{temp_file}')", flush=True)
    print(f"[测试2] 参数: enable=True, path='{temp_file}'", flush=True)
    print(f"[测试2] 路径类型: {'文件' if os.path.isfile(temp_file) else '目录' if os.path.isdir(temp_file) else '不存在'}", flush=True)
    print(f"[测试2] 文件大小: {os.path.getsize(temp_file)} bytes", flush=True)

    file_rejected = False
    file_rejected_txt = ""
    try:
        ret = ctrl.setLogPath(True, temp_file)
        print(f"[测试2] setLogPath 返回值: {ret}", flush=True)
        file_rejected = False
        file_rejected_txt = f"setLogPath(True, 已存在文件) 返回 {ret}，未抛异常，路径: {temp_file}"
    except Exception as e:
        file_rejected = True
        file_rejected_txt = f"setLogPath 抛异常（预期行为）: {type(e).__name__}: {e}"

    print(f"[测试2] {file_rejected_txt}", flush=True)
    record(results, "setLogPath 指向已存在文件被拒绝", file_rejected,
           "setLogPath(True, 已存在文件) 抛异常或返回错误",
           file_rejected_txt)

    # 清理临时文件
    try:
        os.remove(temp_file)
    except OSError:
        pass

    # 清理
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

    print(f"\n[提示] 测试目录: {parent_dir}", flush=True)
    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()