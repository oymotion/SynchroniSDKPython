# -*- coding: utf-8 -*-
"""MISC-FUNC-009：submit 后台线程执行器——返回值 / 线程身份 / 异常 / 串行。

对应用例：10_底层接口与边界补充.md -> MISC-FUNC-009
可自动化：auto（无需设备）

前置条件：
  - 无（submit 是通用线程调度 API，与设备、蓝牙无关）

流程：
  1) submit 纯函数 -> 返回 Future，result(timeout) 取回正确值
  2) 验证 fn 在后台线程（sensor-runner）执行，非调用方线程
  3) 验证 submit 调用本身不阻塞（fn 内 sleep 0.5s 时，submit 立即返回）
  4) submit 抛异常函数 -> Future 正确传回异常
  5) submit 多个任务 -> 单线程 worker 串行执行（提交顺序 == 完成顺序）
"""

import os
import sys
import time
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)

from sensor import *
from sensor import submit  # 被测公开 API：后台线程执行器（__all__ 中导出）
from common import record


def main():
    ctrl = SensorControllerInstance

    print("=" * 60, flush=True)
    print("MISC-FUNC-009 submit 后台线程执行器", flush=True)
    print("=" * 60, flush=True)
    print(f"sdk version = {ctrl.getVersion()}", flush=True)

    results = []

    # ---- 检查1：submit 返回 Future 且 result() 正确 ----
    f = submit(lambda x, y: x + y, 2, 3)
    val = f.result(timeout=5)
    print(f"\n[检查1] submit(lambda x,y: x+y, 2, 3).result(timeout=5) = {val}", flush=True)
    record(results, "submit 返回 Future 且 result() 正确", val == 5,
           "result() == 5", f"result() == {val}")

    # ---- 检查2：fn 在后台线程执行（非调用方线程）----
    main_tid = threading.get_ident()
    seen = {}

    def capture_thread():
        seen["tid"] = threading.get_ident()
        seen["name"] = threading.current_thread().name

    f = submit(capture_thread)
    f.result(timeout=5)
    in_background = (seen.get("tid") != main_tid)
    print(f"[检查2] 主线程 id={main_tid}, fn 线程 id={seen.get('tid')}, name={seen.get('name')!r}", flush=True)
    record(results, "fn 在后台线程执行（非调用方线程）", in_background,
           "fn 线程 != 调用方线程", f"fn 线程 {seen.get('name')!r} (id={seen.get('tid')})")

    # ---- 检查3：submit 调用本身不阻塞（fn 内 sleep）----
    def slow():
        time.sleep(0.5)
        return "done"

    t0 = time.time()
    f = submit(slow)
    submit_cost = time.time() - t0
    r = f.result(timeout=5)
    non_blocking = (submit_cost < 0.2) and (r == "done")
    print(f"[检查3] submit 调用耗时 {submit_cost:.4f}s（fn 内 sleep 0.5s），result={r!r}", flush=True)
    record(results, "submit 调用不阻塞（立即返回 Future）", non_blocking,
           "submit 调用耗时 < 0.2s 且 result 正确", f"submit 耗时 {submit_cost:.4f}s, result={r!r}")

    # ---- 检查4：fn 抛异常 -> Future 正确传回 ----
    def boom():
        raise ValueError("boom")

    f = submit(boom)
    exc = None
    try:
        f.result(timeout=5)
    except ValueError as e:
        exc = e
    exc_ok = (isinstance(exc, ValueError) and str(exc) == "boom")
    print(f"[检查4] fn 抛 ValueError，fut.result() 抛 -> {type(exc).__name__}: {exc}", flush=True)
    record(results, "异常经 Future 正确传回", exc_ok,
           "result() 抛 ValueError('boom')", f"result() 抛 {type(exc).__name__}({exc})")

    # ---- 检查5：单线程 worker 串行执行（提交顺序 == 完成顺序）----
    order = []

    def task_a():
        time.sleep(0.3)
        order.append("A")

    def task_b():
        order.append("B")

    fa = submit(task_a)
    fb = submit(task_b)
    fa.result(timeout=5)
    fb.result(timeout=5)
    serial_ok = (order == ["A", "B"])
    print(f"[检查5] 串行执行顺序 order = {order}", flush=True)
    record(results, "单线程 worker 串行执行（A 先 B 后）", serial_ok,
           "order == ['A', 'B']", f"order == {order}")

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
    ctrl.terminate()


if __name__ == "__main__":
    main()
