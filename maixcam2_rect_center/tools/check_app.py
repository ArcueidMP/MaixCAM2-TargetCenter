#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MaixCAM 应用交付门禁 —— 《MaixCAM应用代码规范.md》的可执行版本。

用法（在应用根目录下，或任意位置用 --root 指定）：

    python -B tools/check_app.py                # 检查，退出码 0=通过 1=有 FAIL
    python -B tools/check_app.py --manifest     # 检查并打印"应用包内文件清单"
    python -B tools/check_app.py --strict       # 把 WARN 也当 FAIL
    python -B tools/check_app.py --list-checks  # 只列出检查项编号与含义

只依赖标准库（不引 PyYAML）：app.yaml 用极简解析，
排除规则用「路径分量精确匹配 + fnmatch 通配」，与 MaixVision「打包应用 / 应用商店」
以及仓库 tools/pack_zip.py 的口径一致：**只读 app.yaml 的 exclude 段当排除规则**。
"""

from __future__ import annotations

import argparse
import ast
import fnmatch
import io
import os
import re
import struct
import sys
import tokenize
from pathlib import Path

OK, WARN, FAIL = "OK", "WARN", "FAIL"

# ---------------------------------------------------------------- 检查项定义

CHECKS = {
    "C1": "app.yaml 存在、字段齐全、id 等于目录名、version 形如 x.y.z",
    "C2": "icon 存在、是 PNG、尺寸/体积在建议范围内",
    "C3": "main.py / config.py 存在，vision/ 有 __init__.py",
    "C4": "所有将进包的 .py 语法编译通过",
    "C5": "AST 扫描：桌面 API 禁止清单、顶层 maix 导入、帧循环内无条件落盘",
    "C6": "将进包的文件均为 UTF-8 无 BOM 且纯 LF",
    "C7": "源码字符串里无绝对 Windows 路径",
    "C8": "BUILD_ID 存在且被打印；有 dist/main_single.py 时核对一致",
    "C9": "exclude 覆盖 .md / tools / dist / __pycache__ / 调试截图；报告失效条目",
    "C10": "包内清单无禁入类型，合计体积在预算内",
}

# ---------------------------------------------------------------- 结果收集

_RESULTS: list[tuple[str, str, str]] = []


def add(level: str, code: str, msg: str) -> None:
    _RESULTS.append((level, code, msg))


def passed(code: str) -> int:
    return sum(1 for lv, c, _ in _RESULTS if c == code and lv == OK)


def failed(code: str) -> int:
    return sum(1 for lv, c, _ in _RESULTS if c == code and lv == FAIL)


def warn_count(code: str) -> int:
    return sum(1 for lv, c, _ in _RESULTS if c == code and lv == WARN)


# ---------------------------------------------------------------- app.yaml

def parse_app_yaml(text: str) -> dict:
    """极简 app.yaml 解析：顶层 `key: value`、`key[zh]: value`、`exclude:` 列表。

    只认本项目实际用到的语法（不引 PyYAML，与仓库既有口径一致）。
    """
    data: dict = {"exclude": []}
    in_exclude = False
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if in_exclude and stripped.startswith("- ") and indent > 0:
            item = stripped[2:].strip().strip("'\"")
            if item:
                data["exclude"].append(item)
            continue
        m = re.match(r"^([A-Za-z_][A-Za-z0-9_\[\]-]*)\s*:\s*(.*)$", stripped)
        if not m:
            in_exclude = False
            continue
        key, val = m.group(1), m.group(2).strip()
        if key == "exclude":
            in_exclude = True
            data["exclude"] = []
            continue
        in_exclude = False
        data[key] = val.strip("'\"")
    return data


# ---------------------------------------------------------------- 打包清单

ALWAYS_EXCLUDED = {".git", "__pycache__", ".gitignore", ".gitattributes"}
FORBIDDEN_PACKED_SUFFIX = {
    ".pyc", ".pyo", ".zip", ".tar", ".gz", ".log", ".md", ".txt",
    ".jpg", ".jpeg", ".bmp", ".json", ".csv", ".xlsx",
}
DEBUG_IMAGE_RE = re.compile(
    r"(screenshot|screen_?check|debug|dbg|mask|frame|cap|_dbg|negative|positive|"
    r"result|overlay|out)[-_0-9a-z]*\.(png|jpg|jpeg|bmp)$", re.I)


def excluded_by(rel: str, is_dir: bool, rules: list[str]) -> bool:
    """rules 里的条目命中 rel 即视为被排除。

    命中方式：① 任一路径分量与条目同名；② 相对路径对条目做 fnmatch；
              ③ 条目以 '/' 结尾时按目录前缀匹配。
    """
    parts = [p for p in rel.split("/") if p]
    for r in rules:
        r = r.strip().strip("'\"")
        if not r:
            continue
        r = r.rstrip("/")
        if r in parts:
            return True
        if fnmatch.fnmatch(rel, r) or fnmatch.fnmatch(rel, r + "/*"):
            return True
        if r.startswith("*.") and fnmatch.fnmatch(parts[-1], r):
            return True
    return False


def build_manifest(root: Path, rules: list[str]) -> tuple[list[tuple[str, int]], list[str], list[str]]:
    """返回 (包内文件列表[(相对路径, 字节)], 被排除的目录, 被排除的文件)。"""
    packed: list[tuple[str, int]] = []
    ex_dirs: list[str] = []
    ex_files: list[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = os.path.relpath(dirpath, root).replace("\\", "/")
        if rel_dir == ".":
            rel_dir = ""
        keep = []
        for d in sorted(dirnames):
            rel = (rel_dir + "/" + d).lstrip("/")
            if d in ALWAYS_EXCLUDED or excluded_by(rel, True, rules):
                ex_dirs.append(rel)
            else:
                keep.append(d)
        dirnames[:] = keep
        for f in sorted(filenames):
            rel = (rel_dir + "/" + f).lstrip("/")
            if f in ALWAYS_EXCLUDED or excluded_by(rel, False, rules):
                ex_files.append(rel)
                continue
            try:
                size = (root / rel).stat().st_size
            except OSError:
                size = -1
            packed.append((rel, size))
    return packed, ex_dirs, ex_files


# ---------------------------------------------------------------- PNG

def png_size(path: Path):
    try:
        head = path.read_bytes()[:33]
    except OSError:
        return None
    if len(head) < 24 or head[:8] != b"\x89PNG\r\n\x1a\n" or head[12:16] != b"IHDR":
        return None
    return struct.unpack(">II", head[16:24])


# ---------------------------------------------------------------- 文本与 Token

def read_bytes(path: Path):
    try:
        return path.read_bytes()
    except OSError:
        return None


def check_text_file(path: Path) -> None:
    raw = read_bytes(path)
    if raw is None:
        add(FAIL, "C6", "%s 读取失败" % path)
        return
    if raw.startswith(b"\xef\xbb\xbf"):
        add(FAIL, "C6", "%s 带 UTF-8 BOM（必须无 BOM）" % path)
    else:
        add(OK, "C6", "%s 无 BOM" % path)
    if b"\r\n" in raw:
        n = raw.count(b"\r\n")
        add(FAIL, "C6", "%s 含 %d 处 CRLF（必须纯 LF）" % (path, n))
    elif b"\r" in raw:
        add(FAIL, "C6", "%s 含裸 CR（必须纯 LF）" % path)
    else:
        add(OK, "C6", "%s 纯 LF" % path)
    try:
        raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        add(FAIL, "C6", "%s 不是合法 UTF-8: %s" % (path, exc))


WIN_PATH_RE = re.compile(r"^[A-Za-z]:[\\/]|^\\\\\\\\[^\\\\]+\\\\")


def check_windows_paths(path: Path, src: str) -> None:
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(src).readline))
    except (tokenize.TokenError, IndentationError) as exc:
        add(FAIL, "C4", "%s 分词失败: %s" % (path, exc))
        return
    bad = []
    for tok in toks:
        if tok.type == tokenize.STRING:
            try:
                val = ast.literal_eval(tok.string)
            except Exception:
                val = tok.string
            if isinstance(val, str) and WIN_PATH_RE.match(val.strip()):
                bad.append((tok.start[0], val[:60]))
    if bad:
        for ln, val in bad[:5]:
            add(FAIL, "C7", "%s:%d 含绝对 Windows 路径 %r" % (path, ln, val))
    else:
        add(OK, "C7", "%s 无绝对 Windows 路径" % path)


# ---------------------------------------------------------------- AST 检查

BANNED_CALLS = {
    "cv2.imshow": "桌面窗口 API（板上无窗口系统）",
    "cv2.namedWindow": "桌面窗口 API",
    "cv2.waitKey": "桌面窗口 API",
    "cv2.destroyAllWindows": "桌面窗口 API",
    "cv2.destroyWindow": "桌面窗口 API",
    "cv2.moveWindow": "桌面窗口 API",
    "cv2.resizeWindow": "桌面窗口 API",
    "cv2.imwrite": "写文件（帧循环内会是秒级 flash 开销，必须单独节流+限量+可关）",
}
BANNED_MODULES = {"matplotlib", "pyplot", "tkinter", "PyQt5", "PySide", "PySide2", "PyQt6"}
BANNED_BUILTINS = {"input", "raw_input"}
LOSSLESS_SINK_HINT = ("imwrite",)  # imencode returns memory; it does not write flash.

# AST 解析失败（语法错误）时的兜底：至少把"禁止项"粗筛出来，不要静默跳过 C5。
_FALLBACK_PATTERNS = [(re.compile(p), msg) for p, msg in [
    (r"\bcv2\.imshow\b", "桌面窗口 API（板上无窗口系统）"),
    (r"\bcv2\.(namedWindow|waitKey|destroyAllWindows|destroyWindow|moveWindow|resizeWindow)\b",
     "桌面窗口 API"),
    (r"\bcv2\.imwrite\b", "写文件（帧循环内会是秒级 flash 开销）"),
    (r"^\s*(?:from|import)\s+(matplotlib|tkinter|PyQt5|PySide2?|PyQt6)\b",
     "导入了桌面/绘图库"),
    (r"(?<![\w.])input\s*\(", "调用了 input()（板上无 stdin）"),
]]


def fallback_scan(path: Path, src: str) -> None:
    """语法坏掉时用正则兜底扫一遍禁止项（宁可粗，不可漏）。"""
    hits = 0
    for i, line in enumerate(src.splitlines(), 1):
        for pat, msg in _FALLBACK_PATTERNS:
            if pat.search(line):
                add(FAIL, "C5", "%s:%d [正则兜底] %s —— %s" % (path, i, line.strip()[:60], msg))
                hits += 1
    if "maix" in src and re.search(r"^\s*from\s+maix\s+import", src, re.M) and \
            not re.search(r"^def\s+|^class\s+", src, re.M):
        add(WARN, "C5", "%s 疑似顶层导入 maix（正则兜底判定，请人工确认）" % path)
    if hits == 0:
        add(WARN, "C5", "%s 语法错误，禁止项只能正则粗筛，未能确认" % path)


def dotted(node: ast.AST) -> str:
    parts = []
    cur = node
    while isinstance(cur, ast.Attribute):
        parts.append(cur.attr)
        cur = cur.value
    if isinstance(cur, ast.Name):
        parts.append(cur.id)
    return ".".join(reversed(parts))


def check_ast(path: Path, src: str, top_level_maix_ok: bool) -> None:
    try:
        tree = ast.parse(src, filename=str(path))
    except SyntaxError as exc:
        add(FAIL, "C4", "%s 语法错误: %s" % (path, exc))
        fallback_scan(path, src)
        return
    add(OK, "C4", "%s 语法编译通过" % path)

    # ---- 顶层 maix 导入（必须放在 main() 内或 try/except 内）
    if top_level_maix_ok:
        for node in tree.body:
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            if any(n == "maix" or n.startswith("maix.") for n in names):
                add(FAIL, "C5", "%s:%d 顶层导入 maix（会破坏 PC 离线单测；应放进 main() 或 try/except）"
                    % (path, node.lineno))

    # ---- 禁止模块
    for node in ast.walk(tree):
        mods = []
        if isinstance(node, ast.Import):
            mods = [a.name.split(".")[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods = [node.module.split(".")[0]]
        for m in mods:
            if m in BANNED_MODULES:
                add(FAIL, "C5", "%s:%d 导入了桌面/绘图库 '%s'" % (path, node.lineno, m))

    # ---- 禁止调用
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = dotted(node.func)
        if name in BANNED_CALLS:
            lv = WARN if name == "cv2.imwrite" else FAIL
            add(lv, "C5", "%s:%d 调用了 %s —— %s" % (path, node.lineno, name, BANNED_CALLS[name]))
        elif isinstance(node.func, ast.Name) and node.func.id in BANNED_BUILTINS:
            add(FAIL, "C5", "%s:%d 调用了 %s()（板上无 stdin）" % (path, node.lineno, node.func.id))

    # ---- 帧循环（while/for）里的落盘调用
    for fn in [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        for loop in [n for n in ast.walk(fn) if isinstance(n, (ast.While, ast.For))]:
            for call in [n for n in ast.walk(loop) if isinstance(n, ast.Call)]:
                nm = dotted(call.func)
                if any(h in nm for h in LOSSLESS_SINK_HINT):
                    guarded = any(isinstance(p, ast.If) for p in ast.walk(loop))
                    lv = WARN if guarded else FAIL
                    add(lv, "C5", "%s:%d 在 %s() 的循环内落盘（%s）—— 需确认已节流+限量+可关闭%s"
                        % (path, call.lineno, fn.name, nm, "（检测到有 If 保护）" if guarded else "（未见任何条件保护）"))


def defines_class(tree_or_src) -> bool:
    return "class Config" in tree_or_src


# ---------------------------------------------------------------- 主流程

def resolve_root(arg_root, script_dir: Path) -> Path:
    if arg_root:
        return Path(arg_root).resolve()
    # tools/ 的上一级就是应用根
    cand = script_dir.parent
    if (cand / "app.yaml").exists():
        return cand.resolve()
    cur = Path.cwd()
    return cur.resolve()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(add_help=True, description="MaixCAM 应用交付门禁")
    ap.add_argument("--root", default=None, help="应用根目录（含 app.yaml）；默认取脚本所在目录的上一级")
    ap.add_argument("--manifest", action="store_true", help="打印应用包内文件清单")
    ap.add_argument("--strict", action="store_true", help="把 WARN 也视为 FAIL")
    ap.add_argument("--list-checks", action="store_true", help="列出检查项编号与含义后退出")
    args = ap.parse_args(argv)

    if args.list_checks:
        for k in sorted(CHECKS):
            print("%s  %s" % (k, CHECKS[k]))
        return 0

    script_dir = Path(__file__).resolve().parent
    root = resolve_root(args.root, script_dir)
    name = root.name

    print("=" * 78)
    print("MaixCAM 应用门禁  root = %s" % root)
    print("=" * 78)

    app_yaml = root / "app.yaml"
    if not app_yaml.exists():
        add(FAIL, "C1", "缺少 app.yaml（应用包根目录必须有且仅有一个）")
        rules, cfg = [], {}
    else:
        text = app_yaml.read_text(encoding="utf-8", errors="replace")
        cfg = parse_app_yaml(text)
        rules = cfg.get("exclude", [])
        add(OK, "C1", "app.yaml 存在")
        # 嵌套 app.yaml 检测
        nested = []
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in ALWAYS_EXCLUDED]
            for f in filenames:
                if f == "app.yaml" and Path(dirpath) != root:
                    nested.append(str(Path(dirpath, f).relative_to(root)).replace("\\", "/"))
        if nested:
            add(FAIL, "C1", "存在嵌套 app.yaml：%s —— 必须写进 exclude 或移出应用根" % ", ".join(nested))
        else:
            add(OK, "C1", "无嵌套 app.yaml")

    required = ["id", "name", "version", "icon", "author", "desc"]
    missing = [k for k in required if not cfg.get(k)]
    if missing:
        add(FAIL, "C1", "app.yaml 缺少必需字段: %s" % ", ".join(missing))
    else:
        add(OK, "C1", "必需字段齐全: %s" % ", ".join(required))
    for k in ("name[zh]", "desc[zh]"):
        if not cfg.get(k):
            add(WARN, "C1", "app.yaml 建议补充 %s（MaixVision 中文显示）" % k)
        else:
            add(OK, "C1", "%s 已填" % k)
    if cfg.get("id"):
        if cfg["id"] != name:
            add(FAIL, "C1", "app.yaml 的 id='%s' 与目录名 '%s' 不一致（会导致设备上无法判断跑的是谁）"
                % (cfg["id"], name))
        else:
            add(OK, "C1", "id 与目录名一致: %s" % name)
    if cfg.get("version") and not re.match(r"^\d+\.\d+\.\d+$", str(cfg["version"])):
        add(WARN, "C1", "version='%s' 不是 x.y.z 形式" % cfg["version"])

    # ---------------- C2 icon
    icon = cfg.get("icon")
    if not icon:
        add(FAIL, "C2", "app.yaml 未声明 icon")
    else:
        ip = root / icon
        if not ip.exists():
            add(FAIL, "C2", "icon 指向的文件不存在: %s" % icon)
        else:
            size = png_size(ip)
            if size is None:
                add(WARN, "C2", "icon 不是标准 PNG: %s" % icon)
            else:
                w, h = size
                kb = ip.stat().st_size / 1024.0
                if w != h:
                    add(WARN, "C2", "icon 非正方形 %dx%d（建议正方形）" % (w, h))
                if max(w, h) > 512:
                    add(WARN, "C2", "icon 尺寸 %dx%d 偏大（建议 <=256x256）" % (w, h))
                if kb > 64:
                    add(WARN, "C2", "icon 体积 %.0fKB 偏大（建议 <=64KB）" % kb)
                add(OK, "C2", "icon PNG %dx%d %.1fKB" % (w, h, kb))

    # ---------------- C3 关键文件
    mainpy = root / "main.py"
    if not mainpy.exists():
        add(FAIL, "C3", "缺少 main.py（launcher 按 main.sh -> main.py -> <app_id> 找入口）")
    else:
        add(OK, "C3", "main.py 存在")
    cfgpy = root / "config.py"
    if not cfgpy.exists():
        add(WARN, "C3", "缺少 config.py（建议把现场可调参数集中到该文件）")
    else:
        add(OK, "C3", "config.py 存在")
    vision = root / "vision"
    if vision.is_dir():
        if not (vision / "__init__.py").exists():
            add(FAIL, "C3", "vision/ 存在但没有 __init__.py（板上无法当作包 import）")
        else:
            add(OK, "C3", "vision/__init__.py 存在")
    else:
        add(WARN, "C3", "没有 vision/ 子包（算法建议与设备 IO 分离，便于 PC 单测）")

    # ---------------- 清单
    packed, ex_dirs, ex_files = build_manifest(root, rules)

    # ---------------- C4 / C5 / C6 / C7 逐文件
    py_files, text_files = [], []
    for rel, _sz in packed:
        low = rel.lower()
        p = root / rel
        if low.endswith(".py"):
            py_files.append(rel)
            raw = read_bytes(p) or b""
            try:
                src = raw.decode("utf-8")
            except UnicodeDecodeError:
                continue  # C6 已报
            check_ast(p, src, top_level_maix_ok=(rel == "main.py"))
            check_windows_paths(p, src)
        if low.endswith((".py", ".yaml", ".yml")):
            text_files.append(rel)
    if not py_files:
        add(FAIL, "C4", "包内没有任何 .py（exclude 可能排错了）")
    for rel in text_files:
        check_text_file(root / rel)

    # ---------------- C8 BUILD_ID
    if mainpy.exists():
        msrc = mainpy.read_text(encoding="utf-8", errors="replace")
        m = re.search(r"^BUILD_ID\s*=\s*[\"']([^\"']+)[\"']", msrc, re.M)
        if not m:
            add(FAIL, "C8", "main.py 未定义 BUILD_ID（无法判断板子跑的是否新代码）")
        else:
            bid = m.group(1)
            if msrc.count("BUILD_ID") < 2:
                add(FAIL, "C8", "main.py 定义了 BUILD_ID='%s' 但没有在启动日志里打印" % bid)
            else:
                add(OK, "C8", "main.py BUILD_ID='%s' 且被引用 %d 次" % (bid, msrc.count("BUILD_ID")))
            single = root / "dist" / "main_single.py"
            if single.exists():
                ssrc = single.read_text(encoding="utf-8", errors="replace")
                m2 = re.search(r"^BUILD_ID\s*=\s*[\"']([^\"']+)[\"']", ssrc, re.M)
                if not m2:
                    add(WARN, "C8", "dist/main_single.py 存在但取不到 BUILD_ID")
                elif m2.group(1) != bid:
                    add(FAIL, "C8", "单文件版 BUILD_ID='%s' 与源码 '%s' 不一致（重跑 build_single.py）"
                        % (m2.group(1), bid))
                else:
                    add(OK, "C8", "单文件版 BUILD_ID 与源码一致")

    # ---------------- C9 exclude 覆盖
    packed_names = [r for r, _ in packed]
    must_exclude_dirs = ["tools", "dist", ".workbuddy", ".vscode", "build"]
    for d in must_exclude_dirs:
        dpath = root / d
        if not dpath.exists():
            continue
        if any(r == d or r.startswith(d + "/") for r in packed_names):
            add(FAIL, "C9", "目录 '%s' 会进应用包 —— 必须写进 exclude" % d)
        else:
            add(OK, "C9", "'%s' 已排除" % d)
    md_packed = [r for r in packed_names if r.lower().endswith(".md")]
    if md_packed:
        add(FAIL, "C9", "文档会进应用包: %s —— 必须写进 exclude" % ", ".join(md_packed))
    else:
        add(OK, "C9", "无 .md 进包")
    img_packed = [r for r in packed_names
                  if r.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")) and r != icon]
    for r in img_packed:
        if DEBUG_IMAGE_RE.search(r):
            add(FAIL, "C9", "调试图片会进应用包: %s —— 必须写进 exclude" % r)
        else:
            add(WARN, "C9", "图片会进应用包: %s（若非图标，请排除）" % r)

    # 失效的 exclude 条目。
    # 只有"看起来是个具体文件名、但目录里根本不存在"才值得报 —— 那才是"掩盖真正泄漏"的元凶；
    # 通配模式与常见防御性目录名（dist/build/.vscode/...）属正当预留，不报。
    DEFENSIVE = {"dist", "build", "out", "tmp", "temp", ".vscode", ".idea", ".workbuddy",
                 ".gitignore", ".gitattributes", ".git", "__pycache__", "node_modules",
                 ".pytest_cache", ".mypy_cache", "assets"}
    all_rel = [r for r, _ in packed] + ex_dirs + ex_files
    for rule in rules:
        r = rule.strip().strip("'\"").rstrip("/")
        if not r:
            continue
        hit = any(fnmatch.fnmatch(x, r) or fnmatch.fnmatch(x, r + "/*")
                  or x == r or x.startswith(r + "/") or r in x.split("/")
                  or fnmatch.fnmatch(os.path.basename(x), r) for x in all_rel)
        if hit:
            add(OK, "C9", "exclude 条目 '%s' 有效" % rule)
        elif "*" in r or "?" in r or r in DEFENSIVE:
            add(OK, "C9", "exclude 条目 '%s' 为预留模式（当前无匹配，正常）" % rule)
        else:
            add(WARN, "C9", "exclude 条目 '%s' 指向不存在的文件（历史残留）。"
                            "留着它本身无害，但会掩盖'真正该排除的文件没被排除'，建议清理" % rule)

    # ---------------- C10 清单与禁入类型
    if not packed:
        add(FAIL, "C10", "应用包为空（exclude 写过头了）")
    total = 0
    for rel, sz in packed:
        total += max(0, sz)
        suffix = Path(rel).suffix.lower()
        if suffix in FORBIDDEN_PACKED_SUFFIX:
            add(FAIL, "C10", "包内出现禁入类型: %s" % rel)
    if total > 4 * 1024 * 1024:
        add(WARN, "C10", "应用包合计 %.1fMB 偏大（建议 <1MB）" % (total / 1048576.0))
    else:
        add(OK, "C10", "应用包 %d 个文件，合计 %.1fKB" % (len(packed), total / 1024.0))
    if "main.py" not in packed_names:
        add(FAIL, "C10", "包内没有 main.py")
    if "app.yaml" not in packed_names:
        add(FAIL, "C10", "包内没有 app.yaml（打包时成员必须在 zip 根目录，不能再套一层）")

    # ---------------- 输出
    # 保证每个检查项都有结论：否则"没报错"与"根本没检查"分不清。
    seen = {c for _, c, _ in _RESULTS}
    for c in CHECKS:
        if c not in seen:
            add(OK, c, "未发现任何问题")
    by_code: dict[str, list[tuple[str, str]]] = {}
    for lv, code, msg in _RESULTS:
        by_code.setdefault(code, []).append((lv, msg))

    n_fail = sum(1 for lv, _, _ in _RESULTS if lv == FAIL)
    n_warn = sum(1 for lv, _, _ in _RESULTS if lv == WARN)
    n_ok = sum(1 for lv, _, _ in _RESULTS if lv == OK)

    for code in sorted(by_code):
        items = by_code[code]
        nf = sum(1 for lv, _ in items if lv == FAIL)
        nw = sum(1 for lv, _ in items if lv == WARN)
        head = "FAIL" if nf else ("WARN" if nw else "OK  ")
        print("\n[%s] %s  %s" % (head, code, CHECKS[code]))
        for lv, msg in items:
            if lv == OK:
                continue
            print("      %-4s %s" % (lv, msg))
        n_okc = sum(1 for lv, _ in items if lv == OK)
        if nf == 0:
            print("      OK   %d 项通过" % n_okc)

    if args.manifest:
        print("\n" + "-" * 78)
        print("应用包内文件清单（%d 项）" % len(packed))
        print("-" * 78)
        base = root.name + "/"
        for rel, sz in packed:
            print("  %8d  %s%s" % (sz, base, rel))
        print("  %8d  TOTAL" % total)
        print("\n被排除的目录（%d）: %s" % (len(ex_dirs), ", ".join(ex_dirs) or "-"))
        exf_show = ex_files[:20]
        print("被排除的文件（%d）: %s%s" % (len(ex_files), ", ".join(exf_show),
                                          " ..." if len(ex_files) > 20 else ""))

    print("\n" + "=" * 78)
    print("汇总: OK=%d  WARN=%d  FAIL=%d   ->  %s" %
          (n_ok, n_warn, n_fail, "通过" if n_fail == 0 else "不通过"))
    print("=" * 78)
    if n_fail:
        return 1
    if args.strict and n_warn:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
