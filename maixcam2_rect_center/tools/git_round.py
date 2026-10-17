#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨道 B「每轮交 git」助手 —— 一条命令完成：自检 → 精确 add → commit。

为什么需要它：本项目两轨共用一个工作树、并发写盘，`maixcam2_rect_center/main.py`
被**整份覆写**过多次（MaixVision 烧录 / 另一个会话 / IDE 未保存标签页 / 并行代理）。
`git` 是目前唯一可靠的回滚点，所以**每完成一轮就提交一次**（详见本目录 AGENTS.md §5）。

本脚本把"每轮交 git"压成一条命令，并**默认只 add 本轨目录**
（`maixcam2_rect_center/`），**绝不使用 `git add -A`**，避免卷入另一条轨的半成品。

用法（在 maixcam2_rect_center/ 下，本机 Python 用 "D:/Python311/python.exe"）：
    python -B tools/git_round.py --check
        只做检查：BUILD_ID 一致性 / build_single --check / check_app --manifest / 待提交清单。
    python -B tools/git_round.py -m "gpt: 摘要"
        先跑上面的检查；通过后精确 add 本轨目录并 commit。
    python -B tools/git_round.py -m "gpt: 摘要" --with-memory
        连 .workbuddy/memory/ 一起提交（跨会话记忆；默认不含，避免卷入别人的记忆改动）。
    python -B tools/git_round.py --rollback main.py
        打印 main.py 的提交历史与"回滚到某次提交"的命令（**默认不执行**）。
    python -B tools/git_round.py --rollback main.py --apply <sha>
        真正执行 `git checkout <sha> -- <文件>`（需显式给 sha）。
"""

import io
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)              # maixcam2_rect_center/
REPO = os.path.dirname(ROOT)              # 25Efork/（仓库根）
SELF_DIR = os.path.basename(ROOT)         # "maixcam2_rect_center"
MEM_DIR = ".workbuddy/memory"
BUILD_RE = r'^BUILD_ID = "([^"]*)"'
PREFIXES = ("gpt:", "repo:", "docs:", "tools:", "code:", "deepseek:")


def run(cmd, cwd=None):
    """跑一条命令，返回 (returncode, stdout+stderr 文本)。"""
    p = subprocess.run(cmd, cwd=cwd or REPO, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT)
    out = p.stdout.decode("utf-8", "replace")
    return p.returncode, out


def _read_build_id(path):
    if not os.path.isfile(path):
        return None
    for line in io.open(path, encoding="utf-8", errors="replace"):
        m = re.match(BUILD_RE, line)
        if m:
            return m.group(1)
    return None


def checks(verbose=True):
    """跑一遍自检，返回 (all_ok, warnings)。"""
    ok = True
    warns = []

    if verbose:
        print("=== [1/4] BUILD_ID 一致性（main.py vs dist/main_single.py）===")
    src_build = _read_build_id(os.path.join(ROOT, "main.py"))
    dis_build = _read_build_id(os.path.join(ROOT, "dist", "main_single.py"))
    if verbose:
        print("  main.py : %s" % src_build)
        print("  dist    : %s" % dis_build)
    if src_build is None:
        ok = False
        warns.append("main.py 里找不到 BUILD_ID")
    if src_build != dis_build:
        ok = False
        warns.append("dist 与 main.py 的 BUILD_ID 不一致 -> 先跑 tools/build_single.py")

    if verbose:
        print("\n=== [2/4] 单文件版是否过期（build_single --check）===")
    rc, out = run([sys.executable, "-B", "tools/build_single.py", "--check"], cwd=ROOT)
    if verbose:
        print("  " + "\n  ".join(out.strip().splitlines()))
    if rc != 0:
        ok = False
        warns.append("build_single --check 未通过 -> 先跑 tools/build_single.py")

    if verbose:
        print("\n=== [3/4] 当前模块化应用门禁（check_app --manifest）===")
    rc, out = run([sys.executable, "-B", "tools/check_app.py", "--manifest"], cwd=ROOT)
    if verbose:
        print(out.strip())
    if rc != 0:
        ok = False
        warns.append("应用门禁未通过")

    if verbose:
        print("\n=== [4/4] 本轨待提交清单（git status --short）===")
    rc, out = run(["git", "status", "--short", "--", SELF_DIR])
    if verbose:
        print(out.strip() or "  (无改动)")
    return ok, warns


def do_commit(message, with_memory):
    ok, warns = checks()
    if not ok:
        print("\n[git_round] 自检未通过，已放弃提交：")
        for w in warns:
            print("  - " + w)
        print("[git_round] 修好后重试，或确认无误后手工 commit。")
        return 2

    if not message.startswith(PREFIXES):
        message = "gpt: " + message
        print("[git_round] 提交信息自动加前缀 -> %s" % message)

    paths = [SELF_DIR]
    if with_memory:
        paths.append(MEM_DIR)

    rc, out = run(["git", "diff", "--cached", "--name-only", "-z"])
    unrelated = [p for p in out.split("\0") if p and not any(p == root or p.startswith(root+"/") for root in paths)]
    if rc != 0 or unrelated:
        print("[git_round] 暂存区含本轮路径以外的改动，停止提交：" + repr(unrelated))
        return 3
    rc, out = run(["git", "add", "--"] + paths)
    if rc != 0:
        print("[git_round] git add 失败：\n" + out)
        return 3

    rc, out = run(["git", "diff", "--cached", "--name-only", "--"] + paths)
    staged = [l for l in out.splitlines() if l.strip()]
    if not staged:
        print("[git_round] 暂存区为空（本轨无改动？），不产生提交。")
        return 0
    print("[git_round] 将提交 %d 个路径：" % len(staged))
    for l in staged:
        print("  " + l)

    rc, out = run(["git", "commit", "-m", message])
    print(out.strip())
    if rc != 0:
        return rc

    rc, out = run(["git", "log", "--oneline", "-1"])
    print("[git_round] 已提交：" + out.strip())
    print("[git_round] 回滚提示：git checkout <sha> -- <文件>（见 AGENTS.md §5.4）")
    return 0


def do_rollback(relpath, apply_sha=None):
    # git 命令以**仓库根**为 cwd，故路径必须是仓库相对的：统一补上 "maixcam2_rect_center/"。
    relpath = relpath.replace("\\", "/").lstrip("./")
    if not relpath.startswith(SELF_DIR + "/") and relpath != SELF_DIR:
        relpath = SELF_DIR + "/" + relpath
    target = relpath
    print("=== %s 的提交历史（最近 15 条）===" % target)
    rc, out = run(["git", "log", "--oneline", "-15", "--", target])
    print(out.strip() or "  (无历史)")

    if apply_sha:
        rc, out = run(["git", "checkout", apply_sha, "--", target])
        print("\n[git_round] git checkout %s -- %s" % (apply_sha, target))
        print(out.strip() or "  (完成)")
        print("[git_round] 恢复后**必须**重跑：tools/build_single.py --check 与 "
              "tools/check_app.py --manifest（C8 会核对 BUILD_ID 一致）")
        return rc

    print("\n[git_round] 回滚命令（按需选一条，本脚本不会自动执行）：")
    print("  git checkout <sha> -- %s        # 把该文件恢复到某次提交" % target)
    print("  git revert <sha>                # 撤销某次提交（保留历史，更安全）")
    print("  （要本脚本执行：加 --apply <sha>）")
    return 0


def main(argv):
    check_only = "--check" in argv
    with_memory = "--with-memory" in argv

    message = None
    for i, a in enumerate(argv):
        if a in ("-m", "--message") and i + 1 < len(argv):
            message = argv[i + 1]

    if "--rollback" in argv:
        i = argv.index("--rollback")
        relpath = argv[i + 1] if i + 1 < len(argv) else "main.py"
        sha = None
        if "--apply" in argv:
            j = argv.index("--apply")
            sha = argv[j + 1] if j + 1 < len(argv) else None
        return do_rollback(relpath, sha)

    if check_only or message is None:
        ok, warns = checks()
        print("\n[git_round] 自检结果：%s" % ("通过 ✅" if ok else "未通过 ❌"))
        for w in warns:
            print("  - " + w)
        if message is None:
            print("\n[git_round] 想提交请加 -m \"gpt: 摘要\"；只想检查就加 --check。")
        return 0 if ok else 1

    return do_commit(message, with_memory)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
