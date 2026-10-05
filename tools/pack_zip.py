#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打 zip 分发包（PC 端即可，不依赖 maixtool / MaixVision）。

    python tools/pack_zip.py --src            # 完整源码包（备份 / 交接）
    python tools/pack_zip.py --app            # MaixCAM 应用包（严格按 app.yaml 的 exclude）
    python tools/pack_zip.py --src --app      # 两个都打
    python tools/pack_zip.py --src -o DIR     # 指定输出目录（默认 dist/）

设计要点
- 只解析 app.yaml 的 exclude 段（不引 PyYAML 依赖），保证应用包与 MaixVision
  「打包应用」的口径一致 —— 改 app.yaml 排除项 = 改这个包的内容。
- 任何情况下都排除 __pycache__ / *.pyc / .git / .workbuddy / .vscode，
  免得把 36K 的项目记忆、pyc 带进交付包。
- 只打「视觉代码」相关文件：src 包不含 reference/（upstream 参考资料）与 *.jpg
  （调试图，可用 tools/ 里的脚本随时重生成），所以整个包只有几百 KB，
  不会把电赛目录里那些大文件（建模/报销/小车代码）卷进来。
"""

import argparse
import os
import re
import sys
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# 永远排除（目录名 / 文件名 / 后缀）
ALWAYS_DIRS = {"__pycache__", ".git", ".svn", ".hg", ".workbuddy", ".vscode", ".idea"}
ALWAYS_NAME_HITS = (".DS_Store", "Thumbs.db", "~$")
ALWAYS_SUFFIX = (".pyc", ".pyo")

# src 包额外排除（这些不是本工程的视觉代码 / 可随手重生成）
SRC_EXTRA_DIRS = {"reference", "build", "dist_old"}
SRC_EXTRA_SUFFIX = (".jpg", ".jpeg", ".png.bak")

# app 包的兜底排除（app.yaml 读失败时用）
APP_FALLBACK_EXCLUDE = [".vscode", ".workbuddy", "dist", "build", "tools", "reference",
                        "maixcam2_rect_center", "README.md"]

# app 包无论 app.yaml 怎么写都不进包：
#   · reference/            是 upstream 参考资料，不是应用代码
#   · maixcam2_rect_center/ 是**另一个自带 app.yaml 的独立应用**，
#     嵌套进来会被当成本应用的普通文件一起打包（实测混进 7 个文件）
APP_ALWAYS_EXCLUDE_DIRS = {"reference", "maixcam2_rect_center"}
# 应用包里不该出现文档（README、分析报告之类）
APP_ALWAYS_EXCLUDE_SUFFIX = (".md",)


def read_yaml_list(path, key):
    """从 yaml 文本里抠出 `key:` 下的 `- xxx` 列表（够用就行，不引 yaml 依赖）。"""
    if not path.exists():
        return []
    out, inside = [], False
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if not inside:
            if re.match(r"^%s\s*:" % re.escape(key), line):
                inside = True
                # 支持 `exclude: [a, b]` 内联写法
                inline = line.split(":", 1)[1].strip()
                if inline.startswith("["):
                    return [s.strip().strip("'\"") for s in inline.strip("[]").split(",") if s.strip()]
            continue
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        m = re.match(r"^\s*-\s*(.+?)\s*$", line)
        if m:
            out.append(m.group(1).strip().strip("'\""))
            continue
        # 退出 exclude 段（回到顶格的新键）
        if not line.startswith((" ", "\t")):
            break
    return out


def collect(exclude_dirs, exclude_files, extra_dirs=frozenset(), extra_suffix=()):
    """返回 (abs_path, arcname) 列表，按路径排序，结果稳定可复现。"""
    items = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        d = Path(dirpath)
        rel = d.relative_to(ROOT)
        top = rel.parts[0] if rel.parts else ""
        # 剪枝
        dirnames[:] = [
            n for n in sorted(dirnames)
            if n not in ALWAYS_DIRS and n not in extra_dirs
            and not (not rel.parts and n in exclude_dirs)
        ]
        if top in extra_dirs or top in ALWAYS_DIRS:
            continue
        for fn in sorted(filenames):
            if fn in exclude_files or fn in ALWAYS_DIRS:
                continue
            if fn.startswith(ALWAYS_NAME_HITS) or fn.endswith(ALWAYS_SUFFIX + extra_suffix):
                continue
            p = d / fn
            items.append((p, str(p.relative_to(ROOT)).replace(os.sep, "/")))
    return items


def build(zip_path, items, arcprefix=""):
    zip_path.parent.mkdir(parents=True, exist_ok=True)
    total = 0
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for p, arc in items:
            z.write(p, arcprefix + arc)
            total += p.stat().st_size
    return total


def fmt(n):
    return "%.1f KB" % (n / 1024.0) if n < 1024 * 1024 else "%.2f MB" % (n / 1048576.0)


def main():
    ap = argparse.ArgumentParser(description="把工程打成分发 zip（不含电赛目录里的大文件）")
    ap.add_argument("--src", action="store_true", help="完整源码包（备份/交接）")
    ap.add_argument("--app", action="store_true", help="MaixCAM 应用包（按 app.yaml 的 exclude）")
    ap.add_argument("-o", "--out", default=str(ROOT / "dist"), help="输出目录（默认 dist/）")
    args = ap.parse_args()
    if not (args.src or args.app):
        args.src = True  # 不带参数就默认打源码包

    out = Path(os.path.expanduser(args.out)).resolve()
    stamp = time.strftime("%Y-%m-%d")

    app_yaml = ROOT / "app.yaml"
    app_id = "app"
    app_ver = "0.0.0"
    kv = app_yaml.read_text(encoding="utf-8") if app_yaml.exists() else ""
    m = re.search(r"^id\s*:\s*(\S+)", kv, re.M)
    if m:
        app_id = m.group(1)
    m = re.search(r"^version\s*:\s*(\S+)", kv, re.M)
    if m:
        app_ver = m.group(1)

    results = []

    if args.src:
        items = collect(set(SRC_EXTRA_DIRS), set(), set(SRC_EXTRA_DIRS), SRC_EXTRA_SUFFIX)
        zp = out / ("%s_src_%s.zip" % (app_id, stamp))
        raw = build(zp, items, arcprefix="%s_src_%s/" % (app_id, stamp))
        results.append(("源码包", zp, len(items), raw))

    if args.app:
        exc = read_yaml_list(app_yaml, "exclude") or APP_FALLBACK_EXCLUDE
        dirs = {e.strip("/") for e in exc} | APP_ALWAYS_EXCLUDE_DIRS
        items = collect(dirs, {e.strip("/") for e in exc}, APP_ALWAYS_EXCLUDE_DIRS,
                        APP_ALWAYS_EXCLUDE_SUFFIX)
        zp = out / ("%s_v%s.zip" % (app_id, app_ver))
        raw = build(zp, items, arcprefix="")
        results.append(("应用包", zp, len(items), raw))

    print("=" * 68)
    for name, zp, n, raw in results:
        print("  %-6s %2d 个文件  %-9s -> %s" % (name, n, fmt(raw), zp))
    print("-" * 68)
    print("  注：src=完整源码（含 tools/、README、dist/main_single.py）；")
    print("      app=MaixCAM 应用包（含 app.yaml/main.py/config.py/vision/assets）。")
    for name, zp, n, raw in results:
        print("  [%s] 内容：" % name)
        with zipfile.ZipFile(zp) as z:
            for i in sorted(z.namelist()):
                print("      %s" % i)
    print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
