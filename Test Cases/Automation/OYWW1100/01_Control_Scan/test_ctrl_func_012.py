# -*- coding: utf-8 -*-
"""CTRL-FUNC-012：getVersion 返回非空版本号。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-012
可自动化：auto（无需设备、无需人工）

前置条件：
  - 主机(电脑)：无特殊要求
  - 待测设备：本用例【无需】OYWW1100

流程：
  1) SensorController.getVersion() -> 断言不抛异常
  2) 断言返回非空字符串
  3) 断言返回格式为版本号（x.y.z，至少三段数字）
"""

import os
import re
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from common import record


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-012 getVersion 返回非空版本号", flush=True)
    print("=" * 60, flush=True)
    print("（本用例无需待测设备 OYWW1100）", flush=True)

    results = []

    # 检查1：不抛异常
    raised = False
    err = None
    try:
        ver = ctrl.getVersion()
    except Exception as e:
        raised = True
        err = f"{type(e).__name__}: {e}"
        ver = None

    print(f"[检测] SensorController.getVersion() -> {ver!r}", flush=True)
    record(results, "getVersion 调用不抛异常", not raised,
           "调用不抛异常", f"抛异常 {err}" if raised else "无异常")

    # 检查2：返回非空字符串
    record(results, "getVersion 返回非空字符串", isinstance(ver, str) and bool(ver),
           "返回非空字符串", f"返回 {ver!r}")

    # 检查3：返回格式为版本号（x.y.z，至少三段数字，如 0.8.0）
    fmt_ok = isinstance(ver, str) and bool(re.match(r"^\d+\.\d+\.\d+", ver))
    record(results, "getVersion 返回版本号格式", fmt_ok,
           "返回形如 x.y.z 的版本号", f"返回 {ver!r}")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()
