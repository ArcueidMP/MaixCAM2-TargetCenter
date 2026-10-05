#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/smoke_main.py · 上板前的 PC 预检：用 maix 桩模块把主循环整条跑一遍
================================================================================
目的：不接触开发板就查出 API 误用、拼写错误、坐标/形状问题
      （真机调试一次要烧程序 + 连线，PC 上几秒就能跑完）。

桩模块在 tools/maix_stub/maix/，冒充 maix.camera / display / image / app / sys：
    · 相机返回合成的"会缓慢移动的黑框靶纸"灰度图（底部带干扰条）
    · app.need_exit() 跑满 LIMIT 帧后返回 True，让主循环自然退出
    · display.show() 只记录，不真的显示

运行（在工程根目录）
--------------------------------------------------------------------------------
    python tools/smoke_main.py             # 预检模块化工程 main.py
    python tools/smoke_main.py --single    # 预检单文件版 dist/main_single.py

预检项
--------------------------------------------------------------------------------
    0) 源码版本自检 —— 防止"main.py 被旧内容覆盖"这类静默回退
    1) 主循环整条跑通，且逐帧 valid=True（检测真的成功，不是"没报错"）
    2) 故意插入打不开的档位，验证相机自动降级
    3) 模拟"只下发了单个文件"的情形，验证会给出可照做的中文提示（而不是裸报错）
    4) image2cv 各种返回形状（含真机的 (H,W,1)）都要能转成灰度

**上板前请至少跑一次本脚本**：它守住的两类事故（下发了旧文件、灰度形状变了）
都真实发生过，而且在板上表现为同样的 cv2.error，很难一眼看出是代码没同步。

注意：桩模块只验证"逻辑跑得通"，不验证真机相机参数，也不能替代实机联调。
"""

import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_STUB = os.path.join(_HERE, "maix_stub")
_DIST = os.path.join(_ROOT, "dist")

for p in (_STUB, _DIST, _ROOT):              # 桩模块优先，避免误导入真实 maix
    if p not in sys.path:
        sys.path.insert(0, p)

_SINGLE = "--single" in sys.argv

# 灰度形状兼容修复的关键标记：任一缺失，就说明源码被旧版本覆盖了
_FIX_MARKERS = ("_as_gray", "np.squeeze", "diag_line", "debug_dump")


def load_target():
    """返回 (主程序模块, 显示名)。"""
    if _SINGLE:
        out = os.path.join(_DIST, "main_single.py")
        if not os.path.isfile(out):
            raise SystemExit("[smoke] 找不到单文件版，请先运行：python tools/build_single.py")
        # 顺手提醒单文件版是否已过期
        r = subprocess.run([sys.executable, os.path.join(_HERE, "build_single.py"), "--check"],
                           capture_output=True, text=True)
        print((r.stdout or r.stderr).strip())
        import main_single as m
        return m, "dist/main_single.py（单文件版）"
    import main as m
    return m, "main.py（模块化工程）"


def run(app_main, print_every=0.0, show=True, profile=None, profiles=None,
        print_fps=True):
    """跑一遍主循环；返回主循环打印出来的日志文本。"""
    if profile is not None:
        app_main.Config.PROFILE = profile
    if profiles is not None:
        app_main.Config.PROFILES = profiles
    app_main.Config.PRINT_EVERY = print_every
    app_main.Config.PRINT_FPS = print_fps
    app_main.Config.SHOW = show
    app_main.Config.DEBUG_DUMP = False     # 预检不需要往磁盘写图
    import maix.app as stub_app
    stub_app.reset()                      # 桩的退出计数是全局的，每次跑之前清零
    buf = io.StringIO()
    if print_fps:
        with contextlib.redirect_stdout(buf):
            app_main.main()
    else:
        app_main.main()
    return buf.getvalue()


def count_frames(log):
    """(检出帧数, 总帧数)：全部检出才算检测链路真的通。"""
    return log.count("valid=True"), len(re.findall(r"valid=", log))


def _build_id_of(text):
    """从源码文本里取出 BUILD_ID（没有则返回 None）。"""
    m = re.search(r'^BUILD_ID\s*=\s*"([^"]+)"', text, re.M)
    return m.group(1) if m else None


def read_build_id(path):
    """读取某个源码文件里的 BUILD_ID（读不到则返回 None）。"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return _build_id_of(f.read())
    except OSError:
        return None


def check_source_guard():
    """
    源码版本自检 —— 防止"main.py 被旧内容覆盖"这类静默回退。

    真实事故：灰度形状修复已写入并打好包，之后 main.py 被旧版本覆盖，
    设备上仍旧报同一个 cv2.error（'scn' is 1），而单文件版却是新的。
    这里检查**修复的关键标记**是否还在，并核对两个入口的 BUILD_ID 是否一致。
    """
    print("    文件                           BUILD_ID        关键标记")
    targets = [os.path.join(_DIST, "main_single.py")] if _SINGLE else \
              [os.path.join(_ROOT, "main.py")]
    ok = True
    for path in targets:
        rel = os.path.relpath(path, _ROOT)
        if not os.path.isfile(path):
            print("    %-30s %-16s 文件不存在" % (rel, "-"))
            ok = False
            continue
        with open(path, "r", encoding="utf-8") as f:
            src = f.read()
        missing = [m for m in _FIX_MARKERS if m not in src]
        bid = _build_id_of(src) or "-"
        good = not missing
        ok = ok and good
        print("    %-30s %-16s %s" % (rel, bid, "OK" if good else
                                     "FAIL 缺少 %s" % ",".join(missing)))

    # 两个入口的 BUILD_ID 必须一致（单文件版由 main.py 合成而来）
    ids = {read_build_id(os.path.join(_ROOT, "main.py")),
           read_build_id(os.path.join(_DIST, "main_single.py"))}
    ids.discard(None)
    if len(ids) > 1:
        print("    两个入口 BUILD_ID 不一致：%s —— 请重跑 tools/build_single.py"
              % sorted(ids))
        ok = False

    if not ok:
        print("    → 诊断：源码疑似被旧版本覆盖。请重新执行 "
              "python tools/build_single.py，并确认 MaixVision 编辑器里打开的\n"
              "      main.py 不是旧内容（编辑器保存会把磁盘文件也覆盖掉）。")
    return ok


def check_missing_module_hint():
    """把 main.py 单独放进空目录运行，确认会给出可照做的中文提示。"""
    code = ("import sys;sys.argv=['main.py']\n"
            "exec(compile(open('main.py',encoding='utf-8').read(),'main.py','exec'))\n")
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(_ROOT, "main.py"), "r", encoding="utf-8") as f:
            src = f.read()
        with open(os.path.join(d, "main.py"), "w", encoding="utf-8") as f:
            f.write(src)
        env = dict(os.environ, PYTHONPATH=_STUB, PYTHONIOENCODING="utf-8",
                   PYTHONUTF8="1")
        r = subprocess.run([sys.executable, "-c", code], cwd=d, env=env,
                           capture_output=True, text=True)
    ok = ("运行当前文件" in (r.stdout + r.stderr)
          and "运行项目" in (r.stdout + r.stderr))
    print("[smoke] 缺文件时的提示是否可照做：%s" % ("PASS" if ok else "FAIL"))
    if not ok:
        print((r.stdout + r.stderr)[:600])
    return ok


def check_gray_shapes(app_main, H=40, W=60):
    """
    把 image2cv 的返回形状逐个换着来，验证 to_gray 都能给出 HxW uint8。

    这一项是补上真机踩过的坑：MaixCAM2 的 image2cv 对灰度图返回 3 维 (H,W,1)，
    早期桩模块返回 2 维，于是"ndim==2 判灰度"的错误分支在 PC 上永远走不到，
    一上板就 cv2.error: Bad number of channels (scn is 1)。
    """
    import numpy as np
    import maix.image as stub_image

    base = np.full((H, W), 120, np.uint8)
    cases = (
        ("(H,W)    2维灰度", base),
        ("(H,W,1)  3维带哑轴（真机）", base[:, :, None]),
        ("(1,H,W)  前导哑轴", base[None, :, :]),
        ("(H,1,W)  中间哑轴", base[:, None, :]),
        ("(H,W,3)  RGB888", np.repeat(base[:, :, None], 3, 2)),
        ("(H,W,4)  RGBA8888", np.repeat(base[:, :, None], 4, 2)),
        ("(H,W,2)  RGB565 打包", np.repeat(base[:, :, None], 2, 2)),
    )
    real = app_main.image.image2cv

    def fake(img, ensure_bgr=True, copy=True, _a=None):
        """模拟真机行为：ensure_bgr 会把彩色打包格式转成 BGR，灰度则原样返回。"""
        if ensure_bgr and _a.ndim == 3 and _a.shape[2] == 2:
            return np.repeat(_a[:, :, :1], 3, 2)
        return _a

    ok = True
    print("    输入形状                        -> to_gray 结果")
    for name, arr in cases:
        app_main.image.image2cv = (lambda img, ensure_bgr=True, copy=True, _a=arr:
                                   fake(img, ensure_bgr, copy, _a))
        try:
            g = app_main.to_gray(stub_image.Image(base))
            good = (tuple(g.shape) == (H, W) and g.dtype == np.uint8)
            info = "shape=%s dtype=%s" % (tuple(g.shape), g.dtype)
        except Exception as e:
            good = False
            info = "%s: %s" % (type(e).__name__, e)
        app_main.image.image2cv = real
        ok = ok and good
        print("    %-30s -> %-4s %s" % (name, "OK" if good else "FAIL", info))

    # 再把桩切成"2 维固件"，整跑一遍主循环，确认两种形状约定都能跑通
    orig = stub_image.GRAY_AS_3D
    try:
        for flag in (True, False):
            stub_image.GRAY_AS_3D = flag
            log = run(app_main, print_every=0.0, show=False)
            hit, total = count_frames(log)
            good = (total > 0 and hit == total)
            ok = ok and good
            print("    桩 GRAY_AS_3D=%-5s              -> %-4s 检出 %d/%d 帧"
                  % (flag, "OK" if good else "FAIL", hit, total))
    finally:
        stub_image.GRAY_AS_3D = orig
    return ok


def main():
    app_main, title = load_target()
    ok = True

    print("\n===== 预检 0：源码版本自检（防\"main.py 被旧版本覆盖\"）=====")
    ok = check_source_guard() and ok

    print("\n===== 预检 1/4：%s —— 主循环整条跑通 =====" % title)
    import maix.display as stub_display
    log = ""
    try:
        log = run(app_main, print_every=0.0)
    except Exception as e:
        print("[smoke] FAIL：主循环抛出异常 —— %s: %s" % (type(e).__name__, e))
        if "Bad number of channels" in str(e) or "scn" in str(e):
            print("        诊断：灰度图被当成彩色处理了 —— 多半是 main.py 被旧版本覆盖。")
            print("        处理：① 跑 python tools/build_single.py 重新生成；")
            print("              ② 确认 MaixVision 里打开的 main.py 不是旧内容再保存/运行。")
        ok = False
    else:
        hit, total = count_frames(log)
        print(log, end="")
        print("\n[smoke] 检出 %d/%d 帧；Display 创建 %d 个、show() 调用 %d 次"
              % (hit, total, stub_display.Display.instances, stub_display.Display.shows))
        if total == 0 or hit != total:
            print("[smoke] FAIL：检测链路没有做到逐帧有效")
            ok = False

    print("\n===== 预检 2/4：故意插入打不开的档位，验证自动降级 =====")
    _p, _ps = app_main.Config.PROFILE, app_main.Config.PROFILES
    try:
        run(app_main, print_every=0.0, show=False, profile="boom",
            profiles=(("boom", 2560, 1440, "GRAYSCALE", 3, 30, 0.5),
                      ("standard", 1280, 720, "GRAYSCALE", 2, 30, 0.5),
                      ("lowmem", 640, 480, "GRAYSCALE", 2, 30, 0.75),
                      ("lowmem32", 640, 480, "GRAYSCALE", 1, 30, 0.75)),
            print_fps=False)
    finally:                                   # 还原，避免影响后面的预检
        app_main.Config.PROFILE, app_main.Config.PROFILES = _p, _ps

    print("\n===== 预检 3/4：只下发单个文件时的报错提示 =====")
    ok = check_missing_module_hint() and ok

    print("\n===== 预检 4/4：image2cv 各种返回形状都要能转灰度 =====")
    ok = check_gray_shapes(app_main) and ok

    print("\n[smoke] 预检结束：%s" % ("全部 PASS" if ok else "有 FAIL，见上"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
