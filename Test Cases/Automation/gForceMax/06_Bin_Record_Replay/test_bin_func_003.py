# -*- coding: utf-8 -*-
"""BIN-FUNC-003：getBinFileInfo 无效输入返回 None。

对应用例：06_Bin录制回放解析.md -> BIN-FUNC-003
可自动化：auto（离线用例，无需设备/蓝牙）

流程：
  1) 对不存在的路径调 getBinFileInfo -> 预期 None
  2) 对空文件（0 字节）调 getBinFileInfo -> 预期 None
  3) 对无 config 的垃圾文件调 getBinFileInfo -> 预期 None

说明：
  README：getBinFileInfo(file_path) -> Optional[dict]，"文件不存在或无 config record
  时返回 None"。本用例为纯离线负向校验，不依赖设备连接，也不需开启蓝牙。
  三个负向输入覆盖 README 声明的两种返回 None 场景：文件不存在、无 config record。

前置条件：
  - 无（离线文件级校验）
"""

import os
import sys
import tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from common import record


def _call(ctrl, path):
    try:
        return ctrl.getBinFileInfo(path)
    except Exception as e:
        return f"抛异常 {type(e).__name__}: {e}"


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("BIN-FUNC-003 getBinFileInfo 无效输入返回 None", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print("\n[说明] 离线负向校验，无需设备/蓝牙。", flush=True)

    results = []

    work = tempfile.mkdtemp(prefix="sdklog_bin_invalid_")

    # 1) 不存在的路径
    nonexist = os.path.join(work, "no_such_file.bin")
    r1 = _call(ctrl, nonexist)
    ok1 = (r1 is None)
    print(f"\n[1] getBinFileInfo(不存在路径) -> {r1!r}", flush=True)
    record(results, "getBinFileInfo(不存在路径) 返回 None", ok1,
           "返回 None", f"返回 {r1!r}")

    # 2) 空文件（0 字节）
    empty = os.path.join(work, "empty.bin")
    with open(empty, "wb"):
        pass
    r2 = _call(ctrl, empty)
    ok2 = (r2 is None)
    print(f"[2] getBinFileInfo(空文件) -> {r2!r}", flush=True)
    record(results, "getBinFileInfo(空文件) 返回 None", ok2,
           "返回 None", f"返回 {r2!r}")

    # 3) 无 config 的垃圾文件
    garbage = os.path.join(work, "garbage.bin")
    with open(garbage, "wb") as f:
        f.write(b"this is not a valid sensor bin file " * 10)
    r3 = _call(ctrl, garbage)
    ok3 = (r3 is None)
    print(f"[3] getBinFileInfo(无 config 垃圾文件) -> {r3!r}", flush=True)
    record(results, "getBinFileInfo(无 config 文件) 返回 None", ok3,
           "返回 None", f"返回 {r3!r}")

    # 清理临时文件
    try:
        for fn in (nonexist, empty, garbage):
            if os.path.isfile(fn):
                os.remove(fn)
        os.rmdir(work)
    except OSError:
        pass

    try:
        ctrl.terminate()
    except Exception:
        pass

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
