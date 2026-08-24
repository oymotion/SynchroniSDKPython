# -*- coding: utf-8 -*-
"""批量运行 01_Control_Scan 目录下所有用例，生成 txt 总结报告。

- 逐个以子进程运行目录下所有 test_*.py（排除本 runner 自身 test_ctrl_scan_all.py）。
- 子进程 stdin 继承当前终端：原脚本内部的人工操作提示（开关机、插拔 dongle 等）
  由测试人员现场处理，无需在本 runner 里重复提示。
- 捕获每个脚本的完整输出，按末尾「结论: PASS/FAIL」判定结果。
- 运行结束后生成 scan_all_report.txt：总览 + 统计 + 失败/异常脚本的完整输出。
"""

import os
import subprocess
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(BASE_DIR, "scan_all_report.txt")
RUNNER = os.path.basename(__file__)


def _targets():
    names = sorted(
        f for f in os.listdir(BASE_DIR)
        if f.startswith("test_") and f.endswith(".py") and f != RUNNER
    )
    return names


def _run_one(script):
    """运行单个脚本，逐字符回显到终端并累积输出，返回 (output, returncode)。"""
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"

    p = subprocess.Popen(
        [sys.executable, script],
        cwd=BASE_DIR,
        env=env,
        stdin=None,               # 继承终端，供原脚本 input() 交互
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )

    buf = []
    while True:
        ch = p.stdout.read(1)     # 逐字符读，避免 input 无换行提示被 readline 卡住
        if ch == "":
            break
        buf.append(ch)
        sys.stdout.write(ch)
        sys.stdout.flush()

    p.wait()
    return "".join(buf), p.returncode


def _verdict(output):
    # 先判 FAIL 再判 PASS，避免误命中中间出现的 FAIL 字样
    if "结论: FAIL" in output:
        return "FAIL"
    if "结论: PASS" in output:
        return "PASS"
    return "NO_VERDICT"


def main():
    targets = _targets()
    total = len(targets)
    print("=" * 60)
    print("批量运行 01_Control_Scan 用例（共 %d 个）" % total)
    print("=" * 60)
    if total == 0:
        print("未找到任何 test_*.py 用例。")
        return

    rows = []
    for i, name in enumerate(targets, 1):
        script = os.path.join(BASE_DIR, name)
        t0 = time.time()
        print("\n" + "=" * 60)
        print("[%d/%d] 运行 %s" % (i, total, name))
        print("=" * 60)
        output, rc = _run_one(script)
        cost = time.time() - t0
        verdict = _verdict(output)
        rows.append((name, verdict, rc, cost, output))
        print("\n[结果] %s -> %s (%.1fs, rc=%d)" % (name, verdict, cost, rc))

    n_pass = sum(1 for r in rows if r[1] == "PASS")
    n_fail = sum(1 for r in rows if r[1] == "FAIL")
    n_noverdict = sum(1 for r in rows if r[1] == "NO_VERDICT")
    abnormal = [r for r in rows if r[1] == "FAIL" or r[1] == "NO_VERDICT" or r[2] != 0]

    lines = []
    lines.append("=" * 70)
    lines.append("OYWW1100 01_Control_Scan 批量测试报告")
    lines.append("生成时间: %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("=" * 70)
    lines.append("")
    lines.append("[总览]")
    for name, verdict, rc, cost, _out in rows:
        lines.append("  %-28s %-12s %.1fs  rc=%d" % (name, verdict, cost, rc))
    lines.append("")
    lines.append("[统计]")
    lines.append("  总数: %d" % total)
    lines.append("  PASS: %d" % n_pass)
    lines.append("  FAIL: %d" % n_fail)
    lines.append("  无结论/跳过: %d" % n_noverdict)
    lines.append("")

    if abnormal:
        lines.append("=" * 70)
        lines.append("[错误脚本详细输出]")
        lines.append("=" * 70)
        for name, verdict, rc, cost, output in abnormal:
            lines.append("")
            lines.append("-" * 70)
            lines.append("===== %s (%s, rc=%d, %.1fs) =====" % (name, verdict, rc, cost))
            lines.append("-" * 70)
            lines.append(output if output.strip() else "(无输出)")
    else:
        lines.append("无错误脚本。")

    with open(REPORT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print("\n" + "=" * 60)
    print("报告已生成: %s" % REPORT)
    print("PASS=%d FAIL=%d 无结论=%d" % (n_pass, n_fail, n_noverdict))
    print("=" * 60)


if __name__ == "__main__":
    main()
