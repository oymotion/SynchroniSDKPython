# -*- coding: utf-8 -*-
"""CTRL-BND-001：startScan(period_in_ms) 6 点边界。

对应用例：01_控制器与扫描.md -> CTRL-BND-001
可自动化：auto（无需待测设备，仅验证参数边界）

说明：
  本用例只验证 startScan 的 period_in_ms 参数边界，不真正扫描/连接设备。
  由于 startScan 是 C 扩展实现，非法参数可能导致进程级崩溃（segfault），
  单进程内 try/except 无法捕获进程崩溃，因此每个边界值都在【独立子进程】中调用，
  主进程收集 returncode / 输出，从而能捕获"非法值导致崩溃"这类最严重缺陷。

边界值（MAX 假设为 int32 上限，实测若 SDK 有文档化上限请调整）：
  0、1、MAX-1、MAX、-1、MAX+1

判定：
  - 合法值（0/1/MAX-1/MAX）：不抛异常即通过（返回 True/False 记录，False 可能是蓝牙未开启所致）
  - 非法值（-1/MAX+1）：被拒绝（返回 False 或抛异常）即通过；返回 True 判 FAIL
  - 任一边界值导致子进程崩溃（returncode != 0）判 FAIL
"""

import os
import subprocess
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from common import record

MAX_INT32 = 2147483647  # period_in_ms 假设上限（int32 max）
BOUNDARY_VALUES = [0, 1, MAX_INT32 - 1, MAX_INT32, -1, MAX_INT32 + 1]
LEGAL = {0, 1, MAX_INT32 - 1, MAX_INT32}

# 子进程探针：调用 startScan(int(sys.argv[1])) 并打印结果
PROBE = (
    "import sys\n"
    "from sensor import SensorControllerInstance as c\n"
    "try:\n"
    "    r = c.startScan(int(sys.argv[1]))\n"
    "    print('RET=' + repr(r), flush=True)\n"
    "except Exception as e:\n"
    "    print('RAISED=' + type(e).__name__ + ':' + str(e), flush=True)\n"
)


def _probe(value):
    """在子进程调用 startScan(value)，返回 (returncode, stdout, stderr)。"""
    try:
        r = subprocess.run([sys.executable, '-c', PROBE, str(value)],
                           capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return -999, '', '子进程超时（startScan 疑似阻塞）'
    except Exception as e:
        return -998, '', f'子进程启动异常 {type(e).__name__}: {e}'
    return r.returncode, r.stdout or '', r.stderr or ''


def _parse(out):
    """解析探针输出，返回 (kind, value)。kind: 'return'/'raised'/'unknown'。"""
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
    return ('unknown', None)


def main():
    print("=" * 60, flush=True)
    print("CTRL-BND-001 startScan(period_in_ms) 6 点边界", flush=True)
    print("=" * 60, flush=True)
    print(f"MAX 假设为 int32 上限 = {MAX_INT32}（若 SDK 有文档化上限请调整）", flush=True)
    print(f"边界值 = {BOUNDARY_VALUES}", flush=True)
    print("本用例无需待测设备；每个值在独立子进程调用以捕获崩溃。", flush=True)

    results = []

    for value in BOUNDARY_VALUES:
        rc, out, err = _probe(value)
        kind, ret = _parse(out)
        is_legal = value in LEGAL
        tag = "合法" if is_legal else "非法"

        if rc != 0:
            # 进程级崩溃（segfault / 未捕获异常 / 超时）
            detail = f"子进程 returncode={rc}（疑似崩溃）"
            if err:
                detail += f" stderr={err[:200]}"
            print(f"[{tag}] startScan({value}) -> {detail}", flush=True)
            record(results, f"startScan({value}) 不崩溃", False,
                   "进程正常退出（不崩溃）", detail)
        elif kind == 'unknown':
            detail = f"输出无法解析 out={out!r} err={err[:200]!r}"
            print(f"[{tag}] startScan({value}) -> {detail}", flush=True)
            record(results, f"startScan({value}) 结果可解析", False,
                   "输出含 RET= 或 RAISED=", detail)
        elif kind == 'raised':
            print(f"[{tag}] startScan({value}) -> 抛异常 {ret}", flush=True)
            if is_legal:
                record(results, f"startScan({value}) 合法值不抛异常", False,
                       "合法值不抛异常", f"抛异常 {ret}")
            else:
                record(results, f"startScan({value}) 非法值被拒绝", True,
                       "非法值被拒绝（返回 False 或抛异常）", f"抛异常 {ret}")
        else:  # return True/False
            print(f"[{tag}] startScan({value}) -> 返回 {ret}", flush=True)
            if is_legal:
                note = "合法值不抛异常" if ret is True else "合法值不抛异常（返回 False，可能因蓝牙未开启，需关注）"
                record(results, f"startScan({value}) 合法值不抛异常", True,
                       "合法值不抛异常", f"返回 {ret}")
            else:
                if ret is True:
                    record(results, f"startScan({value}) 非法值被拒绝", False,
                           "非法值被拒绝（返回 False 或抛异常）", "返回 True（被错误接受）")
                else:
                    record(results, f"startScan({value}) 非法值被拒绝", True,
                           "非法值被拒绝（返回 False 或抛异常）", f"返回 {ret}")

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


if __name__ == "__main__":
    main()
