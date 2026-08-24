# -*- coding: utf-8 -*-
"""DATA-BND-002：init(_, powerRefreshInterval) 6 点边界。

对应用例：03_数据流.md -> DATA-BND-002
可自动化：auto（设备上电、在范围内为运行前置，测试中无需人工动作）

说明：
  验证 init 的 powerRefreshInterval 参数边界。init 是 C 扩展实现，非法参数可能
  导致进程级崩溃（segfault），单进程 try/except 无法捕获进程崩溃；且 init 依赖
  设备已 Ready。因此每个边界值都在【独立子进程】中完成"scan->connect->Ready->init"
  的完整流程，主进程收集 returncode / 输出，从而隔离崩溃、保证每次 init 状态干净。

边界值（MAX 假设 int32 上限，README 未文档化 powerRefreshInterval 上限，实测若有
文档化上限请调整）：
  0、1、MAX-1、MAX、-1、MAX+1

判定（与 DATA-BND-001 一致，但注意合法下限不同）：
  - 合法值（0/1/MAX-1/MAX）：不崩溃、不抛异常即通过（返回 True/False 均记录；
    MAX-1/MAX 若返回 False 可能是超出 SDK 实际上限，标注"需确认"）
  - 非法值（-1/MAX+1）：被拒绝（返回 False 或抛异常）即通过；返回 True 判 FAIL
  - 任一边界值导致子进程崩溃（returncode != 0）判 FAIL

  注：powerRefreshInterval 合法范围 ≥0，故 0 是合法值；而 packageSampleCount
  合法范围 ≥1，故 DATA-BND-001 中 0 是非法值。两者下限不同。

  前置不满足（设备未扫到/未 Ready 等）会自动重试 3 次（间隔 3s），仍失败则记
  INVALID（不计入 PASS/FAIL，需重跑），避免把环境抖动误判为 SDK 缺陷。
"""

import os
import subprocess
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

import config

MAX_INT32 = 2147483647  # powerRefreshInterval 假设上限（int32 max）
BOUNDARY_VALUES = [0, 1, MAX_INT32 - 1, MAX_INT32, -1, MAX_INT32 + 1]
LEGAL = {0, 1, MAX_INT32 - 1, MAX_INT32}

# 前置不满足（设备未扫到/未 Ready 等）时的重试配置
RETRY_MAX = 3                 # 额外重试次数（共最多尝试 1+3 次）
RETRY_INTERVAL_SEC = 3        # 重试间隔（秒）
PRE_KINDS = {'notarget', 'noprofile', 'notready', 'scanraised', 'connectraised'}

# 子进程探针：完整走 scan->match->connect->Ready->init(package, val) 并打印结果
PROBE = r'''
import sys, time
from sensor import *

val = int(sys.argv[1])          # powerRefreshInterval（本用例的边界变量）
package = int(sys.argv[2])      # packageSampleCount（固定合法值）
target_identity = sys.argv[3].strip().upper()
target_mac = sys.argv[4].strip().upper()
scan_ms = int(sys.argv[5])

def _identity(name):
    name = name or ""
    if "(" in name and ")" in name:
        return name[name.index("(") + 1:name.index(")")].strip().upper()
    return None

ctrl = SensorControllerInstance

try:
    devices = ctrl.scan(scan_ms)
except Exception as e:
    print("SCANRAISED=" + type(e).__name__ + ":" + str(e), flush=True)
    sys.exit(0)

target = None
for d in (devices or []):
    addr = (getattr(d, "Address", "") or "").upper()
    if target_mac and addr == target_mac:
        target = d
        break
    if target_identity and _identity(getattr(d, "Name", "")) == target_identity:
        target = d
        break
if target is None:
    print("NOTARGET", flush=True)
    sys.exit(0)

sensor = ctrl.requireSensor(target)
if sensor is None:
    print("NOPROFILE", flush=True)
    sys.exit(0)

try:
    sensor.connect()
except Exception as e:
    print("CONNECTRAISED=" + type(e).__name__ + ":" + str(e), flush=True)
    sys.exit(0)

t0 = time.time()
while time.time() - t0 < 15 and sensor.deviceState != DeviceStateEx.Ready:
    time.sleep(0.2)

if sensor.deviceState != DeviceStateEx.Ready:
    print("NOTREADY", flush=True)
    sys.exit(0)

try:
    r = sensor.init(package, val)
    print("RET=" + repr(r), flush=True)
except Exception as e:
    print("RAISED=" + type(e).__name__ + ":" + str(e), flush=True)

try:
    sensor.disconnect()
except Exception:
    pass
'''

from common import record, TARGET_IDENTITIES, _find_config


def _target_params():
    """返回本轮目标设备的 (identity, mac)，供 PROBE 匹配。"""
    if not TARGET_IDENTITIES:
        return '', ''
    tid = TARGET_IDENTITIES[0]
    cfg = _find_config(tid)
    mac = (cfg.get("mac") or "").strip().upper() if cfg else ""
    return tid, mac


def _probe(value):
    """在子进程完成 connect+init(package, value)，返回 (returncode, stdout, stderr)。"""
    identity, mac = _target_params()
    args = [sys.executable, '-c', PROBE, str(value),
            str(config.PACKAGE_SAMPLE_COUNT),
            identity, mac, str(config.SCAN_TIMEOUT_MS)]
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=90)
    except subprocess.TimeoutExpired:
        return -999, '', '子进程超时（scan/connect/init 疑似阻塞）'
    except Exception as e:
        return -998, '', f'子进程启动异常 {type(e).__name__}: {e}'
    return r.returncode, r.stdout or '', r.stderr or ''


def _parse(out):
    """解析探针输出，返回 (kind, value)。"""
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('RET='):
            s = line[len('RET='):]
            if s == 'True':
                return ('return', True)
            if s == 'False':
                return ('return', False)
            return ('return', s)
        if line.startswith('RAISED='):
            return ('raised', line[len('RAISED='):])
        if line.startswith('NOTARGET'):
            return ('notarget', None)
        if line.startswith('NOPROFILE'):
            return ('noprofile', None)
        if line.startswith('NOTREADY'):
            return ('notready', None)
        if line.startswith('SCANRAISED='):
            return ('scanraised', line[len('SCANRAISED='):])
        if line.startswith('CONNECTRAISED='):
            return ('connectraised', line[len('CONNECTRAISED='):])
    return ('unknown', None)


def main():
    print("=" * 60, flush=True)
    print("DATA-BND-002 init(_, powerRefreshInterval) 6 点边界", flush=True)
    print("=" * 60, flush=True)
    print(f"MAX 假设为 int32 上限 = {MAX_INT32}（README 未文档化上限，若实际有请调整）", flush=True)
    print(f"边界值 = {BOUNDARY_VALUES}", flush=True)
    print(f"固定 packageSampleCount = {config.PACKAGE_SAMPLE_COUNT}（合法值）", flush=True)
    print(f"目标设备 identity = {config.TARGET_IDENTITY}", flush=True)

    print("\n[前置条件]", flush=True)
    print("  - 主机(电脑)：蓝牙已开启", flush=True)
    print("  - 待测设备：OB6000C 上电、在范围内", flush=True)
    print("  - 每个边界值在独立子进程 connect+init，全程设备保持开机即可", flush=True)

    input("\n>>> [人工操作] 请确认待测设备 OB6000C 已【开机】且在范围内，"
          "测试过程无需额外动作，完成后按回车继续 ...")

    results = []

    for i, value in enumerate(BOUNDARY_VALUES):
        tag = "合法" if value in LEGAL else "非法"
        print(f"\n[{i+1}/{len(BOUNDARY_VALUES)}] 测试 powerRefreshInterval = {value}（{tag}）...", flush=True)

        # 前置不满足时重试（最多额外重试 RETRY_MAX 次，间隔 RETRY_INTERVAL_SEC）
        kind = ret = None
        attempt = 0
        while True:
            attempt += 1
            rc, out, err = _probe(value)
            kind, ret = _parse(out)
            if kind in PRE_KINDS and attempt <= RETRY_MAX:
                print(f"  [重试 {attempt}/{RETRY_MAX}] 前置不满足（{kind}），{RETRY_INTERVAL_SEC}s 后重试 ...", flush=True)
                time.sleep(RETRY_INTERVAL_SEC)
                continue
            break

        if rc != 0:
            detail = f"子进程 returncode={rc}（疑似崩溃）"
            if err:
                detail += f" stderr={err[:200]}"
            print(f"[{tag}] init({value}) -> {detail}", flush=True)
            record(results, f"init({value}) 不崩溃", False,
                   "进程正常退出（不崩溃）", detail)
        elif kind in PRE_KINDS:
            detail = f"前置不满足：{kind}" + (f" {ret}" if ret else "") + f"（重试 {RETRY_MAX} 次仍失败）"
            print(f"[{tag}] init({value}) -> {detail}", flush=True)
            record(results, f"init({value}) 前置条件满足", None,
                   "scan 匹配到目标且 connect 到 Ready", detail)
        elif kind == 'unknown':
            detail = f"输出无法解析 out={out!r} err={err[:200]!r}"
            print(f"[{tag}] init({value}) -> {detail}", flush=True)
            record(results, f"init({value}) 结果可解析", False,
                   "输出含 RET= 或 RAISED=", detail)
        elif kind == 'raised':
            print(f"[{tag}] init({value}) -> 抛异常 {ret}", flush=True)
            if value in LEGAL:
                record(results, f"init({value}) 合法值不抛异常", False,
                       "合法值不抛异常", f"抛异常 {ret}")
            else:
                record(results, f"init({value}) 非法值被拒绝", True,
                       "非法值被拒绝（返回 False 或抛异常）", f"抛异常 {ret}")
        else:  # return True/False
            print(f"[{tag}] init({value}) -> 返回 {ret}", flush=True)
            if value in LEGAL:
                if ret is True:
                    record(results, f"init({value}) 合法值不抛异常", True,
                           "合法值不抛异常", f"返回 {ret}")
                else:
                    note = "返回 False（可能超出 SDK 实际上限，需确认，不判 FAIL）"
                    record(results, f"init({value}) 合法值不抛异常", True,
                           "合法值不抛异常", f"返回 {ret}，{note}")
            else:
                if ret is True:
                    record(results, f"init({value}) 非法值被拒绝", False,
                           "非法值被拒绝（返回 False 或抛异常）", "返回 True（被错误接受）")
                else:
                    record(results, f"init({value}) 非法值被拒绝", True,
                           "非法值被拒绝（返回 False 或抛异常）", f"返回 {ret}")

        # 子进程间留间隙，确保上一轮 disconnect 完成、BLE 后端释放
        if i < len(BOUNDARY_VALUES) - 1:
            time.sleep(2)

    # ---- 汇总 ----
    print("\n" + "=" * 60, flush=True)
    print("测试结果汇总", flush=True)
    print("=" * 60, flush=True)
    all_pass = True
    invalid_count = 0
    for name, status, expect, actual in results:
        if status == "PASS":
            print(f"  [PASS] {name}（实际: {actual}）", flush=True)
        elif status == "INVALID":
            invalid_count += 1
            print(f"  [INVALID] {name}（{actual}）", flush=True)
        else:
            print(f"  [FAIL] {name}", flush=True)
            print(f"         期待: {expect}", flush=True)
            print(f"         实际: {actual}", flush=True)
            all_pass = False

    if invalid_count:
        print(f"\n注意：{invalid_count} 项因前置条件不满足被标记 INVALID（未计入 PASS/FAIL，需重跑）。", flush=True)

    print("\n结论: " + ("PASS" if all_pass else "FAIL"), flush=True)


if __name__ == "__main__":
    main()
