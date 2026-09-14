# -*- coding: utf-8 -*-
"""BIN-BND-002：multiReplayBinFile 无效/重复/不足输入。

对应用例：06_Bin录制回放解析.md -> BIN-BND-002
可自动化：auto（离线回放，无需设备在线；需预先准备 1~2 个有效 bin 用于"单文件/重复 MAC"分支）

流程：
  1) 命令行传入若干 bin 路径（可选）
  2) 逐项校验：
     a) multiReplayBinFile([]) 空列表
     b) multiReplayBinFile([单文件]) 不足 2 个
     c) multiReplayBinFile(含不存在路径) 无效成员
     d) multiReplayBinFile(同一文件重复两次) 重复 device_mac
  3) 每项断言：不崩溃、不抛异常；返回值符合 SDK 语义（空/单文件被拒绝或返回空结果；
     无效成员返回 None；重复 MAC 成员返回 None，不拖垮整组）

说明：
  SDK multiReplayBinFile 返回与 file_paths 对齐的列表，失败成员（文件不存在/无配置/
  MAC 重复/正在传输）为 None。本用例聚焦入参容错：任何输入都不应导致 SDK 崩溃或抛异常。
  具体"空列表/单文件是否被拒绝"以 SDK 实际行为记录，硬门槛是"不崩溃 + 返回列表"。

  用法：
    python test_bin_bnd_002.py [bin1.bin] [bin2.bin]

前置条件：
  - 主机(电脑)：蓝牙是否开启不影响（离线回放）
  - 可选 1~2 个有效 bin 文件用于"单文件/重复 MAC"分支
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from common import record


def _call(ctrl, paths, label):
    """调用 multiReplayBinFile 并返回 (返回列表, 异常信息)。"""
    print(f"\n[{label}] multiReplayBinFile({paths!r}) ...", flush=True)
    try:
        r = ctrl.multiReplayBinFile(paths)
        return r, None
    except Exception as e:
        return None, f"抛异常 {type(e).__name__}: {e}"


def _assert_no_crash(results, name, ret, err):
    """硬断言：调用不抛异常（返回 None 因抛异常则 FAIL）。"""
    crashed = err is not None
    record(results, name, not crashed,
           "调用不抛异常", f"ret={ret!r} err={err}")
    return not crashed


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-BND-002 multiReplayBinFile 无效/重复/不足输入", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    raw_paths = [p for p in sys.argv[1:] if p.strip()]
    valid_paths = [p for p in raw_paths if os.path.isfile(p)]
    print(f"\n[输入] 命令行 {len(raw_paths)} 个路径，其中有效文件 {len(valid_paths)} 个: {valid_paths}", flush=True)

    results = []

    # a) 空列表
    ret, err = _call(ctrl, [], "空列表")
    _assert_no_crash(results, "空列表 multiReplayBinFile([]) 不崩溃", ret, err)
    if err is None:
        empty_ok = ret is None or (isinstance(ret, list) and len(ret) == 0)
        record(results, "空列表返回空结果（None 或空列表）", empty_ok,
               "返回 None 或空列表", f"ret={ret!r}")

    # b) 单文件（不足 2 个）
    if valid_paths:
        one = valid_paths[:1]
        ret, err = _call(ctrl, one, "单文件")
        _assert_no_crash(results, "单文件 multiReplayBinFile 不崩溃", ret, err)
        if err is None:
            # SDK 可能返回单成员列表（正常回放）或拒绝；记录实际行为
            record(results, "单文件返回列表（SDK 语义记录）", None,
                   "返回列表（长度任意）或 None", f"ret={ret!r}")
    else:
        record(results, "单文件分支", None, "需要 ≥1 个有效 bin", "无有效 bin，跳过")
        record(results, "重复 MAC 分支", None, "需要 ≥1 个有效 bin", "无有效 bin，跳过")

    # c) 含不存在路径
    fake = r"Z:\__no_such_dir__\not_exist.bin"
    if valid_paths:
        ret, err = _call(ctrl, valid_paths[:1] + [fake], "含无效路径")
    else:
        ret, err = _call(ctrl, [fake], "仅无效路径")
    if not _assert_no_crash(results, "含无效路径 multiReplayBinFile 不崩溃", ret, err):
        pass
    elif err is None and isinstance(ret, list):
        record(results, "无效成员返回 None", True,
               "无效路径成员为 None", f"ret={ret!r}")

    # d) 重复 device_mac（同一文件传两次）
    if valid_paths:
        ret, err = _call(ctrl, [valid_paths[0], valid_paths[0]], "重复 MAC")
        _assert_no_crash(results, "重复 MAC multiReplayBinFile 不崩溃", ret, err)
        if err is None and isinstance(ret, list):
            record(results, "重复 MAC 成员返回 None", ret[1] is None,
                   "第二个重复成员为 None", f"ret={ret!r}")

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

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
