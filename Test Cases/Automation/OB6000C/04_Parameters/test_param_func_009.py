# -*- coding: utf-8 -*-
"""PARAM-FUNC-009：NEUCIR_*（不适用）。

对应用例：04_参数.md -> PARAM-FUNC-009
可自动化：manual

流程：
  1) OB6000C 非 NeuCir 设备，本用例不适用。
  2) 记录"不适用"并跳过。

前置条件：
  - 无
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
from common import record


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("PARAM-FUNC-009 NEUCIR_*（不适用）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)

    results = []

    record(results, "NEUCIR_* 参数测试", None,
           "NeuCir 设备时执行",
           "OB6000C 非 NeuCir 设备，不适用")
    print("[SKIP] OB6000C 非 NeuCir 设备，NEUCIR_* 参数不适用，跳过", flush=True)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    for rname, status, expect, actual in results:
        print(f"  [SKIP] {rname}（{actual}）", flush=True)

    print("\n结论: SKIP", flush=True)
    ctrl.terminate()


if __name__ == "__main__":
    main()