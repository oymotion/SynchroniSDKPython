# -*- coding: utf-8 -*-
"""DEV-SM-006：Ready 前不发送命令（状态门控）。

对应用例：02_连接与状态机.md -> DEV-SM-006
可自动化：auto（设备上电在范围内为运行前置，测试中无需人工动作）

说明：
  在 Disconnected（未连接）状态下调用 init / setParam / startDataNotification，
  验证 SDK 拒绝这些命令（返回 False/Error，或抛异常）且不崩溃。
  Connecting/Connected 阶段过渡极快，难以脚本捕获，故以 Disconnected 作为
  "Ready 前"的稳定代表状态（同样满足"Ready 前不发送命令"的约束）。

  因这三个命令是 C 扩展实现，若状态门控失效可能 segfault，try/except 捕获不到，
  故把"在 Disconnected 状态调用命令"放在独立子进程执行，主进程以 returncode 判定崩溃。

  主进程不初始化 SensorControllerInstance、不 scan、不读后端，避免占用蓝牙后端
  （bumble dongle 同一 dongle 只能被一个进程打开），全部蓝牙操作交给子进程。

前置条件：
  - 主机(电脑)：蓝牙已开启
  - 待测设备：OYWW1100 上电、在范围内
"""

import os
import subprocess
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

import config


from common import record, TARGET_IDENTITIES, _find_config

# 读取 config 中本轮目标设备（config.TARGET_IDENTITY）对应的匹配参数，供 PROBE format 与 main 使用
TARGET_MAC = ''
TARGET_IDENTITY = ''
if TARGET_IDENTITIES:
    _cfg = _find_config(TARGET_IDENTITIES[0])
    if _cfg:
        TARGET_MAC = (_cfg.get("mac") or "").strip().upper()
        TARGET_IDENTITY = (_cfg.get("identity") or "").strip().upper()


def _key(out, key):
    for line in out.splitlines():
        line = line.strip()
        if line.startswith(key + '='):
            return line[len(key) + 1:]
    return None


# 子进程探针：诊断 -> scan -> 按 config 匹配参数匹配 -> requireSensor -> 在 Disconnected 状态调用三个命令
PROBE = (
    "import sys, re, time\n"
    "sys.path.insert(0, {automation_dir!r})\n"
    "from sensor import *\n"
    "def _identity_of(name):\n"
    "    m = re.search(r'\\(([0-9A-Fa-f]{{4}})\\)', name or '')\n"
    "    return m.group(1).upper() if m else None\n"
    "ctrl = SensorControllerInstance\n"
    "print('VERSION=' + ctrl.getVersion(), flush=True)\n"
    "print('BACKEND=' + ctrl.getBLEBackendName(), flush=True)\n"
    "print('ISENABLE=' + str(ctrl.isEnable), flush=True)\n"
    "target = None\n"
    "for _attempt in range(1, 4):\n"
    "    devices = ctrl.scan({scan_ms})\n"
    "    print('SCANNED=' + str([(getattr(d,'Name','?'), (getattr(d,'Address','') or '').upper()) for d in devices]), flush=True)\n"
    "    for d in devices:\n"
    "        addr = (getattr(d, 'Address', '') or '').upper()\n"
    "        name = getattr(d, 'Name', '') or ''\n"
    "        if {mac!r} and addr == {mac!r}:\n"
    "            target = d\n"
    "            break\n"
    "        if {identity!r} and _identity_of(name) == {identity!r}:\n"
    "            target = d\n"
    "            break\n"
    "    if target is not None:\n"
    "        break\n"
    "    if _attempt < 3:\n"
    "        print('RETRY=' + str(_attempt), flush=True)\n"
    "        time.sleep(10)\n"
    "if target is None:\n"
    "    print('NOTARGET=1', flush=True)\n"
    "    sys.exit(0)\n"
    "sensor = ctrl.requireSensor(target)\n"
    "if sensor is None:\n"
    "    print('NOSENSOR=1', flush=True)\n"
    "    sys.exit(0)\n"
    "print('STATE=' + str(sensor.deviceState), flush=True)\n"
    "try:\n"
    "    print('INIT_RET=' + str(sensor.init({pkg}, {pwr})), flush=True)\n"
    "except Exception as e:\n"
    "    print('INIT_RAISED=' + type(e).__name__, flush=True)\n"
    "try:\n"
    "    print('SET_RET=' + str(sensor.setParam('NTF_EMG', 'ON')), flush=True)\n"
    "except Exception as e:\n"
    "    print('SET_RAISED=' + type(e).__name__, flush=True)\n"
    "try:\n"
    "    print('START_RET=' + str(sensor.startDataNotification()), flush=True)\n"
    "except Exception as e:\n"
    "    print('START_RAISED=' + type(e).__name__, flush=True)\n"
).format(
    automation_dir=AUTOMATION_DIR,
    scan_ms=config.SCAN_TIMEOUT_MS,
    pkg=config.PACKAGE_SAMPLE_COUNT,
    pwr=config.POWER_REFRESH_INTERVAL_MS,
    mac=TARGET_MAC,
    identity=TARGET_IDENTITY,
)


def main():
    print("=" * 60, flush=True)
    print("DEV-SM-006 Ready 前不发送命令（状态门控）", flush=True)
    print("=" * 60, flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：上电、在范围内", flush=True)
    print(f"  - config 启用目标：identity={TARGET_IDENTITY or '(空)'}", flush=True)

    input("\n>>> [人工操作] 请确认上方 config 启用的目标设备（identity）与待测设备一致，"
          "其余设备已在 config 中 enabled=False 关闭，且待测设备已【开机】在范围内，完成后按回车继续 ...")

    results = []

    print(f"\n[配置] 目标匹配参数 mac={TARGET_MAC!r} identity={TARGET_IDENTITY!r}", flush=True)

    # 子进程：Disconnected 状态调用三个命令
    print("[执行] 子进程在 Disconnected 状态调用 init/setParam/startDataNotification ...", flush=True)
    try:
        r = subprocess.run([sys.executable, '-c', PROBE],
                           capture_output=True, text=True, timeout=60)
        rc = r.returncode
        out = r.stdout or ''
        err = r.stderr or ''
    except subprocess.TimeoutExpired:
        rc = None
        out = ''
        err = ''
        print("[执行] 子进程超时（疑似死锁）", flush=True)
    except Exception as e:
        rc = None
        out = ''
        err = ''
        print(f"[执行] 子进程启动异常 {type(e).__name__}: {e}", flush=True)

    # 打印子进程完整输出，便于定位
    print(f"[子进程 stdout]\n{out if out else '(空)'}", flush=True)
    if err:
        print(f"[子进程 stderr]\n{err}", flush=True)

    # 诊断信息
    version = _key(out, 'VERSION')
    backend = _key(out, 'BACKEND')
    print(f"\n[信息] sdk version = {version}", flush=True)
    print(f"[信息] ble backend = {backend}", flush=True)

    # 环境检查（子进程内）
    is_enable = _key(out, 'ISENABLE')
    print(f"[环境检查] SensorController.isEnable = {is_enable}", flush=True)
    if is_enable != 'True':
        print("[跳过] 前置条件不满足：电脑蓝牙未开启。请先开启【电脑】蓝牙后重跑。", flush=True)
        return

    # 判定1：不崩溃
    print(f"[检查1] 子进程 returncode = {rc}", flush=True)
    record(results, "Ready 前调用命令不崩溃", rc == 0,
           "子进程 returncode==0", f"returncode={rc}" + (f" stderr={err[:200]}" if err else ""))

    if rc != 0:
        print("\n结论: FAIL", flush=True)
        return

    # 子进程是否找到目标/拿到 sensor
    if _key(out, 'NOTARGET') is not None:
        record(results, "子进程匹配到目标设备", False, "子进程扫描匹配到目标设备", "NOTARGET")
        print("\n结论: FAIL", flush=True)
        return
    if _key(out, 'NOSENSOR') is not None:
        record(results, "子进程 requireSensor 返回 SensorProfile", False, "返回 SensorProfile", "NOSENSOR")
        print("\n结论: FAIL", flush=True)
        return

    # 判定2：初始状态 Disconnected
    state_raw = _key(out, 'STATE')
    print(f"[检查2] 初始 deviceState = {state_raw}", flush=True)
    is_disconnected = (state_raw is not None and 'Disconnected' in state_raw)
    record(results, "命令调用前 deviceState==Disconnected", is_disconnected,
           "deviceState == Disconnected", f"deviceState = {state_raw}")

    # 判定3：init 被拒绝
    init_ret = _key(out, 'INIT_RET')
    init_raised = _key(out, 'INIT_RAISED')
    print(f"[检查3] init -> {('RET=' + init_ret) if init_ret is not None else ('RAISED=' + init_raised)}", flush=True)
    init_rejected = (init_raised is not None) or (init_ret is not None and init_ret != 'True')
    record(results, "SensorProfile.init 在 Ready 前被拒绝", init_rejected,
           "init 返回 False/抛异常（非 True）",
           f"init -> {('RET=' + init_ret) if init_ret is not None else ('RAISED=' + init_raised)}")

    # 判定4：setParam 被拒绝
    set_ret = _key(out, 'SET_RET')
    set_raised = _key(out, 'SET_RAISED')
    print(f"[检查4] setParam -> {('RET=' + set_ret) if set_ret is not None else ('RAISED=' + set_raised)}", flush=True)
    set_rejected = (set_raised is not None) or (set_ret is not None and set_ret != 'OK')
    record(results, "SensorProfile.setParam 在 Ready 前被拒绝", set_rejected,
           "setParam 返回非 'OK'/抛异常",
           f"setParam -> {('RET=' + set_ret) if set_ret is not None else ('RAISED=' + set_raised)}")

    # 判定5：startDataNotification 被拒绝
    start_ret = _key(out, 'START_RET')
    start_raised = _key(out, 'START_RAISED')
    print(f"[检查5] startDataNotification -> {('RET=' + start_ret) if start_ret is not None else ('RAISED=' + start_raised)}", flush=True)
    start_rejected = (start_raised is not None) or (start_ret is not None and start_ret != 'True')
    record(results, "SensorProfile.startDataNotification 在 Ready 前被拒绝", start_rejected,
           "startDataNotification 返回 False/抛异常（非 True）",
           f"startDataNotification -> {('RET=' + start_ret) if start_ret is not None else ('RAISED=' + start_raised)}")

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    for rname, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {rname}（实际: {actual}）", flush=True)
        else:
            print(f"  [FAIL] {rname}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
        if status != "PASS":
            all_pass = False

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
