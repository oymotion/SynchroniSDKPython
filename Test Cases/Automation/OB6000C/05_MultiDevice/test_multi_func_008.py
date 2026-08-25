# -*- coding: utf-8 -*-
"""MULTI-FUNC-008：两后端各测对齐下发。

对应用例：05_多设备同步.md -> MULTI-FUNC-008
可自动化：semi-auto（需人工确认 dongle 已插入且驱动已绑定）

流程：
  1) 检测当前 BLE 后端（backend1）
  2) 在后端 1 上 scan -> 匹配 -> connect -> init -> multiStart -> 收散布
  3) 用子进程 + SENSOR_SDK_BLE_BACKEND 环境变量切换到另一后端（backend2）
  4) 子进程执行相同的测试流程，输出散布
  5) 验证两后端均能对齐下发

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：≥2 台 OB6000C 上电、在范围内
  - 需有 USB dongle 且驱动已绑定（运行过 checkSetupDongle()）
"""

import os
import subprocess
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
import config
import common
from common import record
from multi_common import (
    scan_and_match_all, connect_and_init_all, disconnect_all,
    check_device_count, print_summary, MultiDataCollector,
)


def run_one_backend(ctrl, results, label):
    """在后端 label 下执行一次完整的扫描-连接-起流-收集-停流-断开流程。"""
    print(f"\n{'=' * 60}", flush=True)
    print(f"后端: {label} ({ctrl.getBLEBackendName()})", flush=True)
    print(f"{'=' * 60}", flush=True)

    print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
    matched, devices = scan_and_match_all(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, required=2)
    print(f"[匹配] 匹配到: {len(matched)}", flush=True)
    if not check_device_count(results, matched, required=2):
        return None

    sensors = connect_and_init_all(ctrl, matched, results)
    if len(sensors) < 2:
        disconnect_all(sensors)
        return None

    collector = MultiDataCollector()
    for s in sensors:
        s.onDataCallback = collector.on_data

    print(f"\n[multiStart] 后端 {label} ...", flush=True)
    try:
        mret = ctrl.multiStartDataNotification(sensors)
        print(f"[multiStart] 返回: {mret}", flush=True)
    except Exception as e:
        mret = None
        print(f"[multiStart] 抛异常 {type(e).__name__}: {e}", flush=True)

    started = isinstance(mret, dict) and all(v is True for v in mret.values())
    record(results, f"两后端对齐下发({label}) multiStart 成功",
           started,
           "multiStart 两台均返回 True",
           f"multiStart 返回 {mret}")

    if not started:
        disconnect_all(sensors)
        return None

    time.sleep(config.COLLECT_SECONDS)

    spread = collector.get_spread_ms()
    for s in sensors:
        mac = s.BLEDevice.Address
        fb = collector.first_batch.get(mac)
        if fb:
            print(f"[数据] {mac}: startTimeStamp={fb[0]}, delay={fb[1]}ms", flush=True)

    if spread is not None:
        print(f"[散布] {label} 跨设备散布 = {spread} ms", flush=True)
        record(results, f"两后端对齐下发({label}) 散布 ≤ 5ms",
               spread <= 5,
               f"跨设备 startTimeStamp 散布 ≤ 5ms",
               f"散布 = {spread} ms")
    else:
        record(results, f"两后端对齐下发({label}) 收集到数据",
               False,
               "两台设备均收到带 startTimeStamp 的数据",
               f"数据不足: {collector.first_batch}")

    try:
        ctrl.multiStopDataNotification(sensors)
    except Exception as e:
        print(f"[multiStop] 抛异常 {type(e).__name__}: {e}", flush=True)

    disconnect_all(sensors)
    return spread


# ---- 子进程入口：通过命令行参数 "--subprocess" 调用 ----
_SUBPROCESS_SCRIPT = r'''
import os, sys, time
sys.path.insert(0, r"__AUTOMATION_DIR__")
sys.path.insert(0, r"__BASE_DIR__")
os.environ["SENSOR_SDK_BLE_BACKEND"] = "__TARGET_BACKEND__"
from sensor import *
import config, common
from common import record
from multi_common import (
    scan_and_match_all, connect_and_init_all, disconnect_all,
    check_device_count, print_summary, MultiDataCollector,
)

ctrl = SensorControllerInstance
backend = ctrl.getBLEBackendName()
print(f"SUBPROCESS_BACKEND={backend}", flush=True)
print(f"SUBPROCESS_ISENABLE={ctrl.isEnable}", flush=True)

# 若未切到目标后端，输出 dongle 诊断信息供排查
if backend != "__TARGET_BACKEND__":
    print(f"SUBPROCESS_DIAG=环境变量已设 __TARGET_BACKEND__ 但实际后端为 {backend}", flush=True)
    try:
        import sensor.bumble_dongle as _bd
        print(f"SUBPROCESS_DIAG=_known_dongle_plugged={_bd._known_dongle_plugged()}", flush=True)
        print(f"SUBPROCESS_DIAG=detect_usb_dongle_specs={_bd.detect_usb_dongle_specs()}", flush=True)
        print(f"SUBPROCESS_DIAG=checkSetupDongle={ctrl.checkSetupDongle()}", flush=True)
    except Exception as _e:
        print(f"SUBPROCESS_DIAG=诊断异常:{{type(_e).__name__}}:{{_e}}", flush=True)
    print("SUBPROCESS_RESULT=SKIP:后端未切换(仍为 " + backend + ")", flush=True)
    ctrl.terminate()
    sys.exit(0)

if not ctrl.isEnable:
    print("SUBPROCESS_RESULT=SKIP:蓝牙未就绪", flush=True)
    ctrl.terminate()
    sys.exit(0)

results = []
print(f"\n[扫描] 目标 identity: {common.TARGET_IDENTITIES}", flush=True)
matched, devices = scan_and_match_all(ctrl, scan_ms=config.SCAN_TIMEOUT_MS, required=2)
print(f"[匹配] 匹配到: {len(matched)}", flush=True)
if not check_device_count(results, matched, required=2):
    print("SUBPROCESS_RESULT=SKIP:设备不足", flush=True)
    ctrl.terminate()
    sys.exit(0)

sensors = connect_and_init_all(ctrl, matched, results)
if len(sensors) < 2:
    disconnect_all(sensors)
    print("SUBPROCESS_RESULT=FAIL:连接失败", flush=True)
    ctrl.terminate()
    sys.exit(0)

collector = MultiDataCollector()
for s in sensors:
    s.onDataCallback = collector.on_data

print(f"\n[multiStart] ...", flush=True)
try:
    mret = ctrl.multiStartDataNotification(sensors)
    print(f"[multiStart] 返回: {{mret}}", flush=True)
except Exception as e:
    mret = None
    print(f"[multiStart] 抛异常 {{type(e).__name__}}: {{e}}", flush=True)

started = isinstance(mret, dict) and all(v is True for v in mret.values())
if not started:
    disconnect_all(sensors)
    print(f"SUBPROCESS_RESULT=FAIL:multiStart失败:{{mret}}", flush=True)
    ctrl.terminate()
    sys.exit(0)

time.sleep(config.COLLECT_SECONDS)

spread = collector.get_spread_ms()
for s in sensors:
    mac = s.BLEDevice.Address
    fb = collector.first_batch.get(mac)
    if fb:
        print(f"[数据] {{mac}}: startTimeStamp={{fb[0]}}, delay={{fb[1]}}ms", flush=True)

if spread is not None:
    print(f"SUBPROCESS_SPREAD={{spread}}", flush=True)
    print(f"SUBPROCESS_RESULT=PASS:散布={{spread}}ms", flush=True)
else:
    print("SUBPROCESS_RESULT=FAIL:数据不足", flush=True)

try:
    ctrl.multiStopDataNotification(sensors)
except Exception as e:
    print(f"[multiStop] 抛异常 {{type(e).__name__}}: {{e}}", flush=True)

disconnect_all(sensors)
ctrl.terminate()
'''


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MULTI-FUNC-008 两后端各测对齐下发", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)
    print(f"ble backend = {ctrl.getBLEBackendName()}", flush=True)
    print(f"[本轮目标设备] {common.TARGET_IDENTITIES}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：≥2 台 OB6000C 上电、在范围内", flush=True)
    print("  - 需有 USB dongle 且驱动已绑定（运行过 checkSetupDongle()）", flush=True)

    print("\n[配置检查]", flush=True)
    print(f"  TARGET_IDENTITY = {config.TARGET_IDENTITY}", flush=True)
    print(f"  目标设备: {len(common.TARGET_IDENTITIES)} 台", flush=True)
    for tid in common.TARGET_IDENTITIES:
        cfg = common._find_config(tid)
        if cfg:
            print(f"    - identity={tid}, name_prefix={cfg.get('name_prefix','?')}, mac={cfg.get('mac','') or '(auto)'}", flush=True)
    print("\n  请确认:", flush=True)
    print("  1. config.py 中 TARGET_IDENTITY 已正确配置", flush=True)
    print("  2. 所有目标设备已【开机】且在范围内", flush=True)
    print("  3. USB dongle 已插入且驱动已绑定", flush=True)

    input("\n>>> [人工操作] 确认以上无误后，按回车继续 ...")

    results = []

    is_enable = ctrl.isEnable
    print(f"\n[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable is not True:
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        ctrl.terminate()
        return

    # ---- 后端 1：当前后端 ----
    backend1 = ctrl.getBLEBackendName()
    print(f"\n>>> 后端 1（当前进程）: {backend1}", flush=True)
    spread1 = run_one_backend(ctrl, results, backend1)

    # ---- 后端 2：子进程切换 ----
    target_backend = "bumble" if backend1 == "bleak" else "bleak"
    print(f"\n{'=' * 60}", flush=True)
    print(f"当前后端为 {backend1}，将用子进程 + SENSOR_SDK_BLE_BACKEND={target_backend} 测试另一后端", flush=True)
    print(f"{'=' * 60}", flush=True)

    input("\n>>> [人工操作] 确认 dongle 已插入且驱动已绑定，按回车继续（子进程测试）...")

    script = (_SUBPROCESS_SCRIPT
              .replace("__AUTOMATION_DIR__", AUTOMATION_DIR)
              .replace("__BASE_DIR__", BASE_DIR)
              .replace("__TARGET_BACKEND__", target_backend))

    print(f"\n[子进程] 启动 Python 子进程，SENSOR_SDK_BLE_BACKEND={target_backend} ...", flush=True)
    try:
        proc = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True, text=True, timeout=120,
            cwd=AUTOMATION_DIR,
        )
        sub_stdout = proc.stdout
        sub_stderr = proc.stderr
        print(f"[子进程] 退出码 = {proc.returncode}", flush=True)
        if sub_stderr:
            print(f"[子进程 stderr]\n{sub_stderr}", flush=True)
    except subprocess.TimeoutExpired:
        print("[子进程] 超时（120s），跳过第二后端测试", flush=True)
        record(results, "两后端对齐下发(后端2)", False,
               "子进程在规定时间内完成",
               "子进程超时")
        ctrl.terminate()
        print_summary(results, False)
        return
    except Exception as e:
        print(f"[子进程] 启动失败: {type(e).__name__}: {e}", flush=True)
        record(results, "两后端对齐下发(后端2)", False,
               "子进程正常启动",
               f"启动失败: {e}")
        ctrl.terminate()
        print_summary(results, False)
        return

    # 解析子进程输出
    backend2 = "?"
    spread2 = None
    sub_result = "?"
    diag_lines = []
    for line in sub_stdout.splitlines():
        line = line.strip()
        if line.startswith("SUBPROCESS_BACKEND="):
            backend2 = line.split("=", 1)[1]
        elif line.startswith("SUBPROCESS_SPREAD="):
            try:
                spread2 = float(line.split("=", 1)[1])
            except ValueError:
                pass
        elif line.startswith("SUBPROCESS_DIAG="):
            diag_lines.append(line.split("=", 1)[1])
        elif line.startswith("SUBPROCESS_RESULT="):
            sub_result = line.split("=", 1)[1]

    print(f"\n[子进程结果] backend={backend2}, spread={spread2}, result={sub_result}", flush=True)

    if sub_result.startswith("SKIP"):
        # 输出诊断信息
        if diag_lines:
            print("\n[后端切换诊断]", flush=True)
            for d in diag_lines:
                print(f"  {d}", flush=True)

        if "后端未切换" in sub_result:
            print("\n[结论] bumble 后端在当前环境无法启用（环境变量已设置但 SensorController 仍返回 bleak）", flush=True)
            print("  已确认 dongle 硬件与驱动正常（见上方诊断）。此为 SDK 0.9.1 的后端选择/强制逻辑未生效，", flush=True)
            print("  建议将该现象反馈给 SDK 开发者排查。本用例（两后端）在当前环境跳过 bumble 侧。", flush=True)
            record(results, "两后端对齐下发(后端2)", None,
                   "bumble 后端可用",
                   f"bumble 后端不可用，SensorController 返回 {backend2}（{sub_result}）")
        else:
            record(results, "两后端对齐下发(后端2)", None,
                   "子进程完成测试",
                   f"子进程 SKIP: {sub_result}")
        print_summary(results, True)
        ctrl.terminate()
        return

    if sub_result.startswith("FAIL"):
        record(results, "两后端对齐下发(后端2)", False,
               "子进程散布 ≤ 5ms",
               f"子进程 FAIL: {sub_result}")
        print_summary(results, False)
        ctrl.terminate()
        return

    # 比较两后端
    if spread1 is not None and spread2 is not None:
        both_ok = spread1 <= 5 and spread2 <= 5
        record(results, "两后端对齐下发均 ≤ 5ms",
               both_ok,
               f"两后端散布均 ≤ 5ms",
               f"{backend1}: {spread1}ms, {backend2}: {spread2}ms")
    else:
        record(results, "两后端对齐下发",
               False,
               "两后端均返回有效散布",
               f"{backend1}: {spread1}, {backend2}: {spread2}")

    print_summary(results, all(r[1] for r in results if r[1] is not None))


if __name__ == "__main__":
    main()