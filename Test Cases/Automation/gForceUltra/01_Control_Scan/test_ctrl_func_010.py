# -*- coding: utf-8 -*-
"""CTRL-FUNC-010：getBLEBackendName 与后端一致（winrt 与 dongle 两种都测）。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-010
可自动化：semi-auto（测 dongle 时需人工关闭主机蓝牙 + 接入 dongle）

说明：
  SDK 1.3.0 后端名与强制方式（README）：
    - winrt  = Windows 原生 BLE；SENSOR_SDK_BLE_BACKEND=winrt 强制
    - dongle = USB BLE dongle 后端；SENSOR_SDK_BLE_BACKEND=dongle 强制
  注意：SDK 会【自动检测】——接入可用 dongle 时默认就切到 dongle 后端。因此必须用
  环境变量显式强制后端，否则"默认后端"会随 dongle 是否接入而变化。
  后端在 SDK 导入时确定，且 SensorControllerInstance 是单例，一个进程内无法切换后端，
  因此本脚本用【子进程】分别以不同环境变量启动 SDK 来测两种后端：
    - 阶段 1（winrt）：SENSOR_SDK_BLE_BACKEND=winrt
    - 阶段 2（dongle）：SENSOR_SDK_BLE_BACKEND=dongle

前置条件：
  - 主机(电脑)：测 dongle 阶段需【关闭主机蓝牙】并【接入 USB BLE dongle】
  - 待测设备：本用例【无需】gForceUltra（不连接传感器）

流程：
  1) 阶段 1：子进程 SENSOR_SDK_BLE_BACKEND=winrt，断言 getBLEBackendName() == "winrt"
  2) 人工关闭主机蓝牙 + 接入 dongle -> 按回车
  3) 阶段 2：子进程 SENSOR_SDK_BLE_BACKEND=dongle，断言 getBLEBackendName() == "dongle"
"""

import os
import subprocess
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from common import record

BACKEND_PROBE = (
    "from sensor import SensorControllerInstance as c; "
    "print('BACKEND=' + str(c.getBLEBackendName()))"
)


def _probe_backend(backend_env):
    """在子进程里以指定后端环境变量启动 SDK，返回 (backend, stderr, returncode)。"""
    env = os.environ.copy()
    env['SENSOR_SDK_BLE_BACKEND'] = backend_env
    try:
        r = subprocess.run([sys.executable, '-c', BACKEND_PROBE],
                           env=env, capture_output=True, text=True, timeout=60)
    except Exception as e:
        return None, f"子进程异常 {type(e).__name__}: {e}", -1
    backend = None
    for line in (r.stdout or '').splitlines():
        if line.startswith('BACKEND='):
            backend = line[len('BACKEND='):].strip()
    return backend, (r.stderr or '').strip(), r.returncode


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("CTRL-FUNC-010 getBLEBackendName 与后端一致（bleak + bumble）", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"当前进程后端 = {ctrl.getBLEBackendName()}", flush=True)
    print("（本用例无需待测设备 gForceUltra）", flush=True)

    results = []

    # ---- 阶段 1：winrt 后端（SENSOR_SDK_BLE_BACKEND=winrt 强制）----
    print("\n[阶段1] 以 SENSOR_SDK_BLE_BACKEND=winrt 启动子进程 ...", flush=True)
    backend, stderr, rc = _probe_backend('winrt')
    print(f"[阶段1] 子进程 getBLEBackendName() = {backend!r} (returncode={rc})", flush=True)
    if stderr:
        print(f"[阶段1] 子进程 stderr: {stderr[:500]}", flush=True)
    record(results, "winrt 后端 getBLEBackendName 返回 winrt", backend == 'winrt',
           'getBLEBackendName() == "winrt"', f"返回 {backend!r}")

    # ---- 阶段 2：dongle 后端（SENSOR_SDK_BLE_BACKEND=dongle 强制）----
    print("\n[阶段2] 准备测 dongle 后端", flush=True)
    print("  - 主机(电脑)：请【关闭】主机蓝牙", flush=True)
    print("  - 主机(电脑)：请【接入】USB BLE dongle", flush=True)
    input(">>> [人工操作] 完成上述两步后按回车继续 ...")

    print("\n[阶段2] 以 SENSOR_SDK_BLE_BACKEND=dongle 启动子进程 ...", flush=True)
    backend, stderr, rc = _probe_backend('dongle')
    print(f"[阶段2] 子进程 getBLEBackendName() = {backend!r} (returncode={rc})", flush=True)
    if stderr:
        print(f"[阶段2] 子进程 stderr: {stderr[:500]}", flush=True)
    record(results, "dongle 后端 getBLEBackendName 返回 dongle", backend == 'dongle',
           'getBLEBackendName() == "dongle"', f"返回 {backend!r}")

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
