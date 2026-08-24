# -*- coding: utf-8 -*-
"""sync_upstream.py — 把 upstream 的最新 SDK 代码 / example / README 同步到本地默认分支。

背景
----
本地仓库的 remote 约定：
  origin   -> 你自己的 fork（jamesxue1982/SynchroniSDKPython）
  upstream -> 官方上游（oymotion/SynchroniSDKPython）

测试代码提交在 Testing_James 分支上。当上游发布新版本（如 0.9.2）时，
本脚本只负责把 upstream 的默认分支（master）同步到本地 master，
不直接操作 Testing_James。之后在 Testing_James 分支上手动执行
`git merge master`（或 `git merge origin/master`）即可把最新代码/example/README
合入测试分支，同时保留测试代码。

用法
----
  python sync_upstream.py          # fetch upstream + merge 到本地 master
  python sync_upstream.py --push   # 同步成功后，把 master 推送到 origin

流程
----
  1. 校验仓库与 upstream remote
  2. git fetch upstream（拉取上游最新）
  3. 确定 upstream 默认分支（master / main）
  4. 检查工作区干净
  5. 切到本地 master
  6. git merge upstream/master（SDK 代码/example/README 随合并带入）
  7. 冲突时列出冲突文件并停在冲突状态；否则报告结果
  8. （可选 --push）git push origin master
"""

import argparse
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def git(args):
    """执行 git 命令，返回 CompletedProcess；统一 UTF-8 输出，避免中文乱码。"""
    cmd = ["git", "-c", "core.quotepath=false",
           "-c", "i18n.logOutputEncoding=utf-8"] + args
    return subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True,
                          encoding="utf-8", errors="replace")


def die(msg):
    print(f"[ERROR] {msg}")
    sys.exit(1)


def show(r):
    """打印命令输出；失败时额外打印 stderr。"""
    if r.stdout.strip():
        print(r.stdout.strip())
    if r.returncode != 0 and r.stderr.strip():
        print(r.stderr.strip())


def main():
    parser = argparse.ArgumentParser(description="同步 upstream 到本地默认分支")
    parser.add_argument("--push", action="store_true",
                        help="同步成功后推送到 origin/<默认分支>")
    args = parser.parse_args()

    print("=" * 60)
    print("同步 upstream -> 本地默认分支")
    print(f"仓库根目录: {REPO_ROOT}")
    print("=" * 60)

    # 1. 确认是 git 仓库
    r = git(["rev-parse", "--is-inside-work-tree"])
    if r.returncode != 0 or r.stdout.strip() != "true":
        die(f"{REPO_ROOT} 不是 git 仓库")

    # 2. 确认 upstream remote
    r = git(["remote", "get-url", "upstream"])
    if r.returncode != 0:
        die("缺少 upstream remote，请先执行：\n"
            "  git remote add upstream https://github.com/oymotion/SynchroniSDKPython.git")
    print(f"upstream: {r.stdout.strip()}")

    # 3. fetch upstream
    print("\n[1/5] git fetch upstream ...")
    r = git(["fetch", "upstream", "--prune"])
    if r.returncode != 0:
        show(r)
        die("fetch upstream 失败")
    print("fetch 完成")

    # 4. 确定 upstream 默认分支名（master / main）
    up_branch = None
    r = git(["symbolic-ref", "refs/remotes/upstream/HEAD"])
    if r.returncode == 0:
        up_branch = r.stdout.strip().split("/")[-1]
    if not up_branch:
        for cand in ("master", "main"):
            rr = git(["rev-parse", "--verify", f"upstream/{cand}"])
            if rr.returncode == 0:
                up_branch = cand
                break
    if not up_branch:
        die("无法确定 upstream 默认分支")
    print(f"upstream 默认分支: {up_branch}")

    # 5. 检查工作区是否干净
    print("\n[2/5] 检查工作区状态 ...")
    r = git(["status", "--porcelain"])
    if r.stdout.strip():
        print("工作区有未提交改动，先处理后再同步：")
        print(r.stdout.strip())
        die("请先 commit 或 stash 后再运行本脚本")

    # 6. 切到本地默认分支
    print(f"\n[3/5] 切换到 {up_branch} ...")
    r = git(["checkout", up_branch])
    if r.returncode != 0:
        show(r)
        die(f"切换到 {up_branch} 失败")
    print(f"当前分支: {up_branch}")

    # 7. merge upstream
    print(f"\n[4/5] 合并 upstream/{up_branch} 到 {up_branch} ...")
    before = git(["rev-parse", "HEAD"]).stdout.strip()
    r = git(["merge", f"upstream/{up_branch}"])
    show(r)
    after = git(["rev-parse", "HEAD"]).stdout.strip()

    if r.returncode != 0:
        conflicts = git(["diff", "--name-only", "--diff-filter=U"]).stdout.strip().splitlines()
        print("\n" + "=" * 60)
        print("合并出现冲突，请手动解决。")
        if conflicts:
            print("冲突文件：")
            for f in conflicts:
                print("  " + f)
        print("解决后执行：git add <文件> && git commit")
        print("如需放弃本次合并：git merge --abort")
        print("=" * 60)
        sys.exit(1)

    if before == after:
        print(f"{up_branch} 已是最新，无需合并（up-to-date）。")
    else:
        print(f"合并完成：{before[:8]} -> {after[:8]}")

    # 8. 可选 push
    if args.push:
        print(f"\n[5/5] 推送 {up_branch} 到 origin ...")
        r = git(["push", "origin", up_branch])
        show(r)
        if r.returncode != 0:
            die("push 失败")
        print("push 完成")
    else:
        print("\n[5/5] 跳过 push（如需推送，加 --push）")

    print("\n同步完成。")


if __name__ == "__main__":
    main()
