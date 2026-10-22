#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从手工用例 Markdown 汇总测试结果到 CSV。

输入一个目录，遍历其中除 README 外的所有 .md 文件，
提取每个测试用例的 ID 与「测试结果」状态，输出为 CSV。

用法：
    python md_to_csv.py <输入目录> [-o 输出.csv]

输出列：文件, 用例ID, 状态
"""

import argparse
import csv
import re
import sys
from pathlib import Path

# 用例标题：### CTRL-FUNC-001 蓝牙未开启时 ...
CASE_RE = re.compile(r"^###\s+([A-Z0-9]+-[A-Z0-9]+-\d+)\b")
# 状态行：- **测试结果**：通过 / 失败，问题 7474 / 跳过（...）
STATUS_RE = re.compile(r"\*\*测试结果\*\*\s*[:：]\s*(.*)$")


def natural_key(name: str):
    """自然排序 key：按数字段拆分，避免 '10' < '2' 的字典序问题。"""
    parts = re.split(r"(\d+)", name)
    return [int(p) if p.isdigit() else p for p in parts]


def parse_md(md_path: Path):
    """解析单个 md，返回 [(用例ID, 状态), ...]，保持文件内顺序。"""
    rows = []
    current_id = None
    current_status = None

    def flush():
        nonlocal current_id, current_status
        if current_id is not None:
            rows.append((current_id, current_status or ""))
        current_id = None
        current_status = None

    for line in md_path.read_text(encoding="utf-8-sig").splitlines():
        m = CASE_RE.match(line.strip())
        if m:
            flush()
            current_id = m.group(1)
            continue
        if current_id is not None:
            m = STATUS_RE.search(line)
            if m:
                current_status = m.group(1).strip()
    flush()
    return rows


def main():
    parser = argparse.ArgumentParser(description="汇总手工用例 Markdown 测试结果为 CSV")
    parser.add_argument("input_dir", help="包含 .md 用例文件的目录")
    parser.add_argument(
        "-o", "--output",
        help="输出 CSV 路径（默认：<输入目录名>_测试结果.csv，落在脚本同目录）",
    )
    args = parser.parse_args()

    in_dir = Path(args.input_dir)
    if not in_dir.is_dir():
        print(f"错误：目录不存在 {in_dir}", file=sys.stderr)
        sys.exit(1)

    md_files = sorted(
        [p for p in in_dir.glob("*.md") if p.stem.lower() != "readme"],
        key=lambda p: natural_key(p.stem),
    )
    if not md_files:
        print(f"警告：{in_dir} 下没有可处理的 .md 文件", file=sys.stderr)
        sys.exit(1)

    out_path = Path(args.output) if args.output else (
        Path(__file__).resolve().parent / f"{in_dir.name}_测试结果.csv"
    )

    total = 0
    with out_path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["文件", "用例ID", "状态"])
        for md in md_files:
            for cid, status in parse_md(md):
                writer.writerow([md.stem, cid, status])
                total += 1

    print(f"处理文件 {len(md_files)} 个，用例 {total} 条")
    print(f"已输出：{out_path}")


if __name__ == "__main__":
    main()
