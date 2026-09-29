# -*- coding: utf-8 -*-
"""CTRL-FUNC-001：蓝牙未开启时 isEnable==False，startScan 被拒绝。

对应用例：01_控制器与扫描.md -> CTRL-FUNC-001
可自动化：semi-auto（需人工关闭/开启蓝牙）

流程（注意：这里的"蓝牙"指【电脑】蓝牙，不是待测设备 OB6000C）：
  1) 人工关闭电脑蓝牙 -> 按回车
     （此时待测设备 OB6000C 保持开机，用于证明"设备在范围内但电脑蓝牙关闭仍扫不到"）
  2) 断言 isEnable == False
  3) 断言 startScan 被拒绝（未进入扫描态 isScanning==False）
  4) 断言 scan 无结果（电脑蓝牙关闭不应扫到设备）
  5) 人工重新开启电脑蓝牙 -> 按回车
     （此阶段只检查电脑蓝牙 isEnable==True，不扫描、不连接设备，
       故 OB6000C 保持开机或关机均可，不影响结果）
  6) 断言 isEnable == True（恢复）

说明（winrt 后端已知问题 7060）：
  电脑蓝牙关闭后，`isEnable` 可能仍返回 True（未感知无线开关），随后 `startScan`
  会触发 native crash，Python 进程直接退出，try/except 无法捕获。因此本脚本把
  阶段 1 的蓝牙操作（isEnable/startScan/scan）放到独立子进程执行，主进程以
  returncode 判定崩溃，避免整个测试进程被杀掉，仍能输出完整判定结果。
  主进程不初始化 SensorControllerInstance，避免占用蓝牙后端。
"""

import os
import subprocess
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

import config
import common
from common import record


def _key(out, key):
    for line in out.splitlines():
        line = line.strip()
        if line.startswith(key + '='):
            return line[len(key) + 1:]
    return None


# 阶段 1 子进程探针：电脑蓝牙关闭时读取 isEnable、startScan、scan
PROBE_OFF = (
    "import sys\n"
    "sys.path.insert(0, {automation_dir!r})\n"
    "from sensor import *\n"
    "ctrl = SensorControllerInstance\n"
    "print('ISENABLE=' + str(ctrl.isEnable), flush=True)\n"
    "try:\n"
    "    print('STARTSCAN_RET=' + str(ctrl.startScan({scan_ms})), flush=True)\n"
    "except Exception as e:\n"
    "    print('STARTSCAN_RAISED=' + type(e).__name__, flush=True)\n"
    "print('ISSCANNING=' + str(ctrl.isScanning), flush=True)\n"
    "try:\n"
    "    ctrl.stopScan()\n"
    "except Exception:\n"
    "    pass\n"
    "try:\n"
    "    _d = ctrl.scan({scan_ms})\n"
    "    print('SCANNED=' + str(len(_d) if _d else 0), flush=True)\n"
    "except Exception as e:\n"
    "    print('SCAN_RAISED=' + type(e).__name__, flush=True)\n"
).format(automation_dir=AUTOMATION_DIR, scan_ms=config.SCAN_TIMEOUT_MS)

# 阶段 2 子进程探针：电脑蓝牙开启时读取 isEnable
PROBE_ON = (
    "import sys\n"
    "sys.path.insert(0, {automation_dir!r})\n"
    "from sensor import *\n"
    "ctrl = SensorControllerInstance\n"
    "print('ISENABLE=' + str(ctrl.isEnable), flush=True)\n"
).format(automation_dir=AUTOMATION_DIR)


def _run(probe, label):
    """在独立子进程运行探针，返回 (returncode, stdout, stderr)。

    returncode == 0 表示子进程正常结束；非 0 表示 native crash；
    None 表示超时或子进程启动失败。
    """
    print(f"[执行] {label}（子进程隔离，防 native crash 波及主进程）...", flush=True)
    try:
        r = subprocess.run([sys.executable, '-c', probe],
                           capture_output=True, text=True, timeout=120)
        return r.returncode, (r.stdout or ''), (r.stderr or '')
    except subprocess.TimeoutExpired:
        print(f"[执行] {label} 子进程超时（疑似死锁）", flush=True)
        return None, '', ''
    except Exception as e:
        print(f"[执行] {label} 子进程启动异常 {type(e).__name__}: {e}", flush=True)
        return None, '', ''


def main():
    print("=" * 60, flush=True)
    print("CTRL-FUNC-001 蓝牙未开启时 isEnable==False，startScan 被拒绝", flush=True)
    print("=" * 60, flush=True)
    print(f"[本轮目标设备] {common.TARGET_IDENTITIES}", flush=True)

    results = []

    # ---- 阶段 1：关闭蓝牙 ----
    input("\n>>> [人工操作] 请【关闭电脑】蓝牙（不是待测设备 OB6000C），并保持 OB6000C 设备开机，完成后按回车继续 ...")

    rc, out, err = _run(PROBE_OFF, "关闭蓝牙时 isEnable/startScan/scan")

    print(f"[子进程 stdout]\n{out if out else '(空)'}", flush=True)
    if err:
        print(f"[子进程 stderr]\n{err}", flush=True)
    if rc is None:
        print("[信息] 子进程未正常结束（超时/启动失败）", flush=True)
    elif rc != 0:
        print(f"[信息] 子进程 native crash（returncode={rc}）", flush=True)

    # 检查 1：isEnable == False
    is_enable = _key(out, 'ISENABLE')
    print(f"\n[检查1] SensorController.isEnable = {is_enable}", flush=True)
    record(results, "蓝牙关闭时 SensorController.isEnable==False", is_enable == 'False',
           "SensorController.isEnable == False", f"SensorController.isEnable == {is_enable}")

    # 检查 2：startScan 被拒绝（不崩溃 + 未进入扫描态）
    startscan_ret = _key(out, 'STARTSCAN_RET')
    startscan_raised = _key(out, 'STARTSCAN_RAISED')
    is_scanning = _key(out, 'ISSCANNING')
    if rc is None:
        check2_ok = False
        check2_actual = "子进程超时/启动失败，无法判定"
    elif rc != 0:
        check2_ok = False
        check2_actual = f"子进程 native crash（returncode={rc}），startScan 触发崩溃而非被拒绝"
    else:
        check2_ok = (is_scanning == 'False')
        call = (f"startScan 返回 {startscan_ret}"
                if startscan_ret is not None
                else f"startScan 抛异常 {startscan_raised}")
        check2_actual = f"{call}, SensorController.isScanning == {is_scanning}"
    print(f"[检查2] SensorController.startScan 被拒绝且未进入扫描态 -> {check2_actual}", flush=True)
    record(results, "SensorController.startScan 被拒绝且未进入扫描态", check2_ok,
           "不崩溃且 SensorController.isScanning == False", check2_actual)

    # 检查 3：scan 无结果
    scanned = _key(out, 'SCANNED')
    scan_raised = _key(out, 'SCAN_RAISED')
    if rc is None:
        check3_ok = False
        check3_actual = "子进程超时/启动失败，无法判定"
    elif rc != 0:
        check3_ok = False
        check3_actual = f"子进程 native crash（returncode={rc}），scan 未正常返回"
    elif scanned is not None:
        check3_ok = (scanned == '0')
        check3_actual = f"scan 返回 {scanned} 台设备"
    else:
        # 抛异常视为无结果（与旧逻辑一致，n=0）
        check3_ok = True
        check3_actual = f"scan 抛异常 {scan_raised}"
    print(f"[检查3] SensorController.scan 无结果（蓝牙关闭）-> {check3_actual}", flush=True)
    record(results, "SensorController.scan 无结果（蓝牙关闭）", check3_ok,
           "SensorController.scan 返回 0 台设备", check3_actual)

    # ---- 阶段 2：重新开启蓝牙 ----
    input("\n>>> [人工操作] 请【开启电脑】蓝牙（不是待测设备 OB6000C），完成后按回车继续 ..."
          "\n    （本阶段只检查电脑蓝牙 isEnable，不扫描、不连接设备；设备 OB6000C 保持开机或关机均可，不影响结果）")
    time.sleep(2)  # 等待系统刷新蓝牙使能状态

    _rc2, out2, err2 = _run(PROBE_ON, "开启蓝牙时 isEnable")
    print(f"[子进程 stdout]\n{out2 if out2 else '(空)'}", flush=True)
    if err2:
        print(f"[子进程 stderr]\n{err2}", flush=True)

    is_enable2 = _key(out2, 'ISENABLE')
    print(f"\n[检查4] 恢复后 SensorController.isEnable = {is_enable2}", flush=True)
    record(results, "恢复后 SensorController.isEnable==True", is_enable2 == 'True',
           "SensorController.isEnable == True", f"SensorController.isEnable == {is_enable2}")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        else:
            print(f"  [{status}] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
