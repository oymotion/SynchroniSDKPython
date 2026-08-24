# -*- coding: utf-8 -*-
"""CTRL-FUNC-011：checkSetupDongle 引导（仅 dongle 后端）。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-011
可自动化：semi-auto（需人工插入 dongle）

前置条件（运行前请人工准备好）：
  - 主机(电脑)：若测 bumble 后端，需插入 USB BLE dongle（Windows 上可能需先
    用管理员权限把 dongle 绑定到 WinUSB 驱动）
  - 待测设备：本用例【无需】OB6000C（只检测 dongle，不连接设备）

流程：
  1) 打印当前后端 getBLEBackendName()
  2) 人工确认 dongle 已插入（bumble 后端时）-> 按回车
  3) checkSetupDongle() -> 断言不抛异常
  4) 断言返回非空字符串
  5) 断言返回格式为 "OK" / "OK: N" / "Error: ..."
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from common import record


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-011 checkSetupDongle 引导", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)

    backend = ctrl.getBLEBackendName()
    print(f"ble backend = {backend}", flush=True)
    print("（本用例无需待测设备 OB6000C，仅检测 USB dongle）", flush=True)

    # 人工操作提示
    if (backend or '').lower() == 'bumble':
        input("\n>>> [人工操作] 请确认 USB BLE dongle 已插入【电脑】（如需装 WinUSB 驱动请先完成），"
              "完成后按回车继续 ...")
    else:
        print("\n当前为 bleak 后端，checkSetupDongle 将退化为 'OK'（无数量的 OK）。", flush=True)
        input(">>> [人工操作] 若想验证 bumble 后端，请设置环境变量 SENSOR_SDK_BLE_BACKEND=bumble、"
              "插入 dongle 后重跑；现在按回车直接在当前后端下测试 ...")

    results = []

    # 调用 checkSetupDongle（可能触发驱动检测/安装脚本，耗时视环境而定）
    print("\n[检测] 正在调用 SensorController.checkSetupDongle() ...（可能耗时）", flush=True)
    raised = False
    err = None
    try:
        ret = ctrl.checkSetupDongle()
    except Exception as e:
        raised = True
        err = f"{type(e).__name__}: {e}"
        ret = None

    print(f"[检测] SensorController.checkSetupDongle() -> {ret!r}", flush=True)

    # 检查1：不抛异常
    record(results, "checkSetupDongle 调用不抛异常", not raised,
           "调用不抛异常", f"抛异常 {err}" if raised else "无异常")

    # 检查2：返回非空字符串
    record(results, "checkSetupDongle 返回非空字符串", isinstance(ret, str) and bool(ret),
           "返回非空字符串", f"返回 {ret!r}")

    # 检查3：返回格式合法（OK / OK: N / Error: ...）
    fmt_ok = isinstance(ret, str) and (ret.startswith('OK') or ret.startswith('Error'))
    record(results, "checkSetupDongle 返回 OK/OK:N/Error 格式", fmt_ok,
           "返回以 OK 或 Error 开头", f"返回 {ret!r}")

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
