# -*- coding: utf-8 -*-
"""CTRL-ROB-001：扫描期间重复 start/stop（并发健壮性）。

对应用例：01_控制器与扫描.md -> CTRL-ROB-001
可自动化：auto（无需待测设备，仅验证扫描状态机并发健壮性）

说明：
  快速交替调用 startScan/stopScan 多次，验证无死锁、无异常、最终 isScanning==False。
  因 startScan/stopScan 是 C 扩展实现，死锁（内部锁）会让脚本卡住、try/except 无法捕获，
  故把重复交替放在【独立子进程】执行，主进程以 timeout 判定死锁。
"""

import os
import subprocess
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from common import record

LOOP_COUNT = 20      # 交替次数
PERIOD_MS = 300      # startScan 短周期
TIMEOUT = 60         # 死锁判定超时（秒）

# 子进程探针：循环 startScan/stopScan，输出异常数、异常列表、最终 isScanning、耗时
PROBE = (
    "import time\n"
    "from sensor import SensorControllerInstance as c\n"
    "t0 = time.time()\n"
    "errs = []\n"
    "for i in range(%d):\n"
    "    try:\n"
    "        c.startScan(%d)\n"
    "    except Exception as e:\n"
    "        errs.append('start#' + str(i) + ':' + type(e).__name__)\n"
    "    try:\n"
    "        c.stopScan()\n"
    "    except Exception as e:\n"
    "        errs.append('stop#' + str(i) + ':' + type(e).__name__)\n"
    "elapsed = round(time.time() - t0, 2)\n"
    "print('ERRS=' + repr(len(errs)), flush=True)\n"
    "print('ERRLIST=' + repr(errs), flush=True)\n"
    "print('ISSCANNING=' + repr(c.isScanning), flush=True)\n"
    "print('ELAPSED=' + str(elapsed), flush=True)\n"
) % (LOOP_COUNT, PERIOD_MS)


def _parse_key(out, key):
    """从探针输出里解析 KEY=value 的 value 字符串，找不到返回 None。"""
    for line in out.splitlines():
        line = line.strip()
        if line.startswith(key + '='):
            return line[len(key) + 1:]
    return None


def main():
    print("=" * 60, flush=True)
    print("CTRL-ROB-001 扫描期间重复 start/stop", flush=True)
    print("=" * 60, flush=True)
    print(f"交替次数 = {LOOP_COUNT}，startScan 周期 = {PERIOD_MS}ms，死锁超时 = {TIMEOUT}s", flush=True)
    print("本用例无需待测设备；重复交替在独立子进程执行，主进程判定死锁。", flush=True)

    results = []

    # 在子进程执行重复交替
    print(f"\n[执行] 子进程循环 {LOOP_COUNT} 次 startScan/stopScan ...", flush=True)
    try:
        r = subprocess.run([sys.executable, '-c', PROBE],
                           capture_output=True, text=True, timeout=TIMEOUT)
        rc = r.returncode
        out = r.stdout or ''
        err = r.stderr or ''
    except subprocess.TimeoutExpired:
        rc = None
        out = ''
        err = ''
        print(f"[执行] 子进程在 {TIMEOUT}s 内未完成（疑似死锁）", flush=True)
        record(results, "重复 start/stop 无死锁", False,
               f"在 {TIMEOUT}s 内完成", f"子进程超时（疑似死锁）")
        print("\n结论: FAIL", flush=True)
        return
    except Exception as e:
        print(f"[执行] 子进程启动异常 {type(e).__name__}: {e}", flush=True)
        record(results, "重复 start/stop 不崩溃", False, "子进程正常启动", f"启动异常 {type(e).__name__}: {e}")
        print("\n结论: FAIL", flush=True)
        return

    # 检查1：不崩溃
    print(f"[检查1] 子进程 returncode = {rc}", flush=True)
    record(results, "重复 start/stop 不崩溃", rc == 0,
           "子进程 returncode==0", f"returncode={rc}" + (f" stderr={err[:200]}" if err else ""))

    if rc != 0:
        print("\n结论: FAIL", flush=True)
        return

    # 解析输出
    errs_raw = _parse_key(out, 'ERRS')
    errlist_raw = _parse_key(out, 'ERRLIST')
    isscanning_raw = _parse_key(out, 'ISSCANNING')
    elapsed_raw = _parse_key(out, 'ELAPSED')

    n_errs = int(errs_raw) if (errs_raw is not None and errs_raw.isdigit()) else None
    is_scanning = (isscanning_raw == 'True')
    print(f"[结果] 异常数 = {n_errs}, 异常列表 = {errlist_raw}", flush=True)
    print(f"[结果] 最终 isScanning = {isscanning_raw}, 循环耗时 = {elapsed_raw}s", flush=True)

    # 检查2：无异常
    if n_errs is None:
        record(results, "重复 start/stop 无异常", False,
               "异常数为 0", f"无法解析异常数 out={out!r}")
    else:
        record(results, "重复 start/stop 无异常", n_errs == 0,
               "异常数为 0", f"异常数 {n_errs}: {errlist_raw}")

    # 检查3：最终 isScanning == False
    record(results, "最终 SensorController.isScanning==False", is_scanning is False,
           "SensorController.isScanning == False", f"isScanning == {isscanning_raw}")

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
