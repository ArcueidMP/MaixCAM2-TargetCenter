#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/build_single.py · 轨道 B：把模块化工程打包成单文件 dist/main_single.py
================================================================================
为什么需要它（和轨道 A 的 tools/build_single.py 同一套理由）
--------------------------------------------------------------------------------
MaixVision 有两种运行方式：
    · 「运行项目」    —— 把整个文件夹打包下发，多文件 import 正常（推荐）
    · 「运行当前文件」—— 只下发当前这一个文件，其它 .py 不会过去，
                        于是报 ModuleNotFoundError: No module named 'config'
本脚本把 config.py + vision/*.py + target_api.py + main.py 合成一个自包含的
main_single.py，让「运行当前文件」也能跑通 —— 现场快速试参数时很省事。

实现原理（语义与分模块时 100% 一致，不改源码一个字）
--------------------------------------------------------------------------------
各模块源码原样嵌入本文件；运行时先造出虚拟模块对象塞进 sys.modules，再 exec 源码。
于是源码里的
    from config import Config                       （main.py）
    from vision.rectangle_center import ...         （main.py / target_api.py）
    from vision.stepper_link import StepperLink     （main.py 函数内）
这些写法不需要任何改写就能正常工作，traceback 里的文件名也仍显示 main.py / vision/*.py。

用法（在 maixcam2_rect_center/ 目录下）
--------------------------------------------------------------------------------
    python tools/build_single.py           # 生成 dist/main_single.py
    python tools/build_single.py --check   # 只检查 dist 里的单文件是否已过期
    python tools/build_single.py --smoke   # PC 离线自检：嵌入的模块能否全部就位

⚠ 改了 config.py / vision/*.py / target_api.py / main.py 之后必须重新生成，
  否则单文件版会停在旧代码（板端日志的 BUILD_ID 会露馅）。
⚠ dist/ 已在 app.yaml 的 exclude 里，**不会进应用包**。
"""

import ast
import hashlib
import io
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "dist")
OUT_PY = os.path.join(OUT_DIR, "main_single.py")

# 需要打进单文件的模块：(模块名, 相对路径) —— 顺序即执行顺序，被依赖的在前
# v0.4.6：入口 main.py 按功能拆分为独立模块，此处同步新增 6 项。
#   · rate_meter / camera_source / vision_worker / preview 依赖少、可直接排在前面；
#   · vision_worker、preview 依赖 rate_meter ⇒ rate_meter 必须在它们之前；
#   · ★ vision.aim_control 依赖 rectangle_center / laser_spot / gimbal_self_test
#     ⇒ 必须排在**这三者之后**（单文件版按序 exec，被依赖者须在前）。
MODULES = (
    ("config", "config.py"),
    ("vision.rate_meter", "vision/rate_meter.py"),
    ("vision.camera_source", "vision/camera_source.py"),
    ("vision.vision_worker", "vision/vision_worker.py"),
    ("vision.telemetry", "vision/telemetry.py"),
    ("vision.preview", "vision/preview.py"),
    ("vision.rectangle_center", "vision/rectangle_center.py"),
    ("vision.laser_spot", "vision/laser_spot.py"),
    ("vision.gimbal_self_test", "vision/gimbal_self_test.py"),
    ("vision.gimbal_aim", "vision/gimbal_aim.py"),
    ("vision.stepper_link", "vision/stepper_link.py"),
    ("vision.aim_control", "vision/aim_control.py"),
    ("vision.gimbal_calibrate", "vision/gimbal_calibrate.py"),
    ("target_api", "target_api.py"),
)
MAIN_REL = "main.py"

_MANIFEST_RE = re.compile(r"^_SRC_SHA1 = (\{.*\})\s*$", re.M)


def sha1_of(path):
    with open(path, "rb") as f:
        return hashlib.sha1(f.read()).hexdigest()


def collect():
    """读取所有源码，返回 (源码字典, sha1 字典)。"""
    src, sha = {}, {}
    for rel in [rel for _, rel in MODULES] + [MAIN_REL]:
        p = os.path.join(ROOT, rel)
        if not os.path.isfile(p):
            raise SystemExit("[build] 找不到源文件: %s" % p)
        with open(p, "r", encoding="utf-8") as f:
            src[rel] = f.read()
        sha[rel] = sha1_of(p)
    return src, sha


def as_raw_literal(text, rel):
    """把源码变成一段安全的 Python 字符串字面量（优先 r'''...'''，保证可读）。

    先补齐末尾换行：这样即使源码最后一行以反斜杠结尾，也不会因为"raw 字符串
    不能以反斜杠收尾"而出错。定界符选源码里没出现过的那一种。
    """
    if not text.endswith("\n"):
        text += "\n"
    for q in ("'''", '"""'):
        if q in text:
            continue
        return "r%s\n%s%s" % (q, text, q)
    raise SystemExit(
        "[build] %s 里同时出现了两种三引号，无法安全嵌入，请改用文件分发方式。" % rel
    )


def build():
    src, sha = collect()
    now = time.strftime("%Y-%m-%d %H:%M:%S")

    buf = io.StringIO()
    w = buf.write
    w('#!/usr/bin/env python3\n')
    w('# -*- coding: utf-8 -*-\n')
    w('"""\n')
    w('main_single.py · 25E 靶心识别（轨道 B）—— 单文件自包含版\n')
    w('=' * 80 + '\n')
    w('★ 本文件由 tools/build_single.py 自动生成，请勿手改！\n')
    w('    改参数 -> config.py ；改算法 -> vision/*.py ；改流程 -> main.py\n')
    w('    改完在本目录重新执行：  python tools/build_single.py\n')
    w('\n')
    w('用途：在 MaixVision 里点「运行当前文件」即可运行（不必用「运行项目」）。\n')
    w('      行为与模块化工程完全一致，输出也就完全一致。\n')
    w('\n')
    w('生成时间：%s\n' % now)
    for k in sorted(sha):
        w('    %-28s sha1=%s\n' % (k, sha[k][:12]))
    w('"""\n')
    w('\n')
    w('import sys as _sys\n')
    w('import types as _types\n')
    w('\n')
    w('_SRC_SHA1 = %r\n' % (sha,))
    w('\n')
    w('# ' + '-' * 74 + '\n')
    w('# ① 各模块源码原样嵌入，运行时装进 sys.modules（保持原 import 语义）\n')
    w('# ' + '-' * 74 + '\n')
    w('_MOD_SRC = {}\n\n')
    for name, rel in MODULES:
        w('# ===== %s  (%s) =====\n' % (name, rel))
        w('_MOD_SRC[%r] = %s\n\n' % (name, as_raw_literal(src[rel], rel)))
    w('_MAIN_SRC = %s\n' % as_raw_literal(src[MAIN_REL], MAIN_REL))
    w('\n')
    w('_MOD_ORDER = (%s)\n\n' % ", ".join(repr(n) for n, _ in MODULES))
    w('_REL_OF = {%s}\n\n' % ", ".join("%r: %r" % (n, rel) for n, rel in MODULES))
    w('_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision\n')
    w('_pkg.__path__ = []\n')
    w('_sys.modules["vision"] = _pkg\n')
    w('for _full in _MOD_ORDER:\n')
    w('    if "." in _full:                       # vision.rectangle_center 这类子模块\n')
    w('        _parent, _leaf = _full.split(".", 1)\n')
    w('        _m = _types.ModuleType(_full)\n')
    w('        setattr(_sys.modules[_parent], _leaf, _m)\n')
    w('        _sys.modules[_full] = _m\n')
    w('    else:                                  # config / target_api 这类顶层模块\n')
    w('        _sys.modules[_full] = _types.ModuleType(_full)\n')
    w('for _full in _MOD_ORDER:\n')
    w('    _m = _sys.modules[_full]\n')
    w('    _m.__dict__["__name__"] = _full\n')
    w('    _m.__dict__["__file__"] = _REL_OF[_full]   # 源码里若用 __file__（如 target_api）\n')
    w('    exec(compile(_MOD_SRC[_full], _full.replace(".", "/") + ".py", "exec"),\n')
    w('         _m.__dict__)\n')
    w('del _m, _full, _MOD_SRC\n')
    w('\n')
    w('# ' + '-' * 74 + '\n')
    w('# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间\n')
    w('#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致\n')
    w('# ' + '-' * 74 + '\n')
    w('exec(compile(_MAIN_SRC, %r, "exec"), globals())\n' % MAIN_REL)

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT_PY, "w", encoding="utf-8", newline="\n") as f:
        f.write(buf.getvalue())
    return sha


def check():
    """检查 dist/main_single.py 是否与当前源码一致。"""
    if not os.path.isfile(OUT_PY):
        print("[check] 还没有生成过 %s" % OUT_PY)
        return 1
    with open(OUT_PY, "r", encoding="utf-8") as f:
        text = f.read()
    m = _MANIFEST_RE.search(text)
    if not m:
        print("[check] 旧产物里没有指纹信息，请重新生成。")
        return 1
    old = ast.literal_eval(m.group(1))
    _, new = collect()
    stale = sorted(k for k in set(old) | set(new) if old.get(k) != new.get(k))
    if stale:
        print("[check] 单文件版已过期，以下源文件在生成之后被改动过：")
        for k in stale:
            print("        - %s" % k)
        print("        请重新执行：python tools/build_single.py")
        return 1
    print("[check] 单文件版是最新的（%d 个源文件指纹一致）。" % len(new))
    return 0


def smoke():
    """PC 离线自检：以非 __main__ 身份执行单文件，确认嵌入模块全部就位。

    只验证"打包是否完整"（模块能否 import、关键符号是否存在、BUILD_ID 是否同步），
    真正的视觉正确性由 tools/test_target_api.py 等回归负责。
    """
    if not os.path.isfile(OUT_PY):
        print("[smoke] 还没有生成过 %s，先跑 python tools/build_single.py" % OUT_PY)
        return 1
    with open(OUT_PY, "r", encoding="utf-8") as f:
        text = f.read()
    ns = {"__name__": "_single_smoke_", "__file__": OUT_PY}
    exec(compile(text, OUT_PY, "exec"), ns)          # 非 __main__ -> 不会真的跑 main()

    wanted = [name for name, _ in MODULES]
    missing = [n for n in wanted if n not in sys.modules]
    if missing:
        print("[smoke] FAIL 这些模块没有被注入 sys.modules: %s" % missing)
        return 1
    from vision.rectangle_center import RectangleCenterDetector     # noqa: F401
    from vision.stepper_link import StepperLink                     # noqa: F401
    from target_api import detect_black_frame, CenterTracker        # noqa: F401
    from config import Config
    checks = []
    with open(os.path.join(ROOT, MAIN_REL), "r", encoding="utf-8") as f:
        main_src = f.read()
    bid_main = re.search(r"^BUILD_ID\s*=\s*[\"']([^\"']+)[\"']", main_src, re.M)
    bid_single = re.search(r"^BUILD_ID\s*=\s*[\"']([^\"']+)[\"']", text, re.M)
    checks.append(("BUILD_ID 与 main.py 一致",
                   bool(bid_main and bid_single and bid_main.group(1) == bid_single.group(1))))
    checks.append(("config 参数可读", bool(getattr(Config, "BLACK_FRAME_MM", None))))
    checks.append(("target_api 符号就位", callable(detect_black_frame) and callable(CenterTracker)))
    bad = [label for label, ok in checks if not ok]
    for label, ok in checks:
        print("  %s %s" % ("OK  " if ok else "FAIL", label))
    if bad:
        print("[smoke] FAIL %s" % bad)
        return 1
    print("[smoke] 单文件版自检通过（%d 个模块已嵌入，BUILD_ID=%s）" %
          (len(wanted), bid_main.group(1) if bid_main else "?"))
    return 0


def main(argv):
    if "--check" in argv:
        return check()
    if "--smoke" in argv:
        return smoke()
    sha = build()
    size = os.path.getsize(OUT_PY)
    print("[build] 已生成 %s" % OUT_PY)
    print("[build] 大小 %.1f KB，包含 %d 个源文件：" % (size / 1024.0, len(sha)))
    for k in sorted(sha):
        print("        %-28s %s" % (k, sha[k][:12]))
    print("[build] 用法：MaixVision 打开该文件后点「运行当前文件」即可。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
