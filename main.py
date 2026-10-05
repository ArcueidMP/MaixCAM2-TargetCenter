#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py · MaixCAM / MaixCAM2 应用入口 —— 靶纸矩形框中心点（靶心）识别
================================================================================
对应赛题：2025 全国大学生电子设计竞赛本科组 E 题《简易自行瞄准装置》

    靶面：A4 紫外感光纸，四周 1.8cm 黑胶带贴出矩形；红色记号笔在中心点靶心
          （≤0.1cm）—— 该点在 165cm 处只有约 0.67px，物理不可见。
    结论：靶心只能由"黑框矩形"几何反算，不能靠找红点。
    输出：靶心像素坐标 (u, v) + 局部尺度 px_per_cm，供云台主控解算 D1。

运行方式（MaixVision）
--------------------------------------------------------------------------------
    · 单文件调试：本文件必须叫 main.py 才能作为应用入口（MaixPy 应用规范）。
    · 项目方式：MaixVision「打开文件夹」选中本工程根目录，点「运行项目」，
      整个文件夹（main.py / config.py / vision/ / app.yaml ...）会一起下发运行。
    · 打包安装：MaixVision 左下角「安装」填写信息后「打包应用 / 安装应用」；
      或命令行 `maixtool release`（产物 dist/<id>_v<ver>.zip），
      再用 `app_store_cli install xxx.zip` 装到板子上，脱离电脑离线运行。

启动时请核对日志里的一行：
        [app] 靶心识别 <BUILD_ID> （根目录 ...）
    版本号与本地 main.py 顶部的 BUILD_ID 一致，才说明板子跑的是最新代码。

内存要点（"帧缓冲内存不足"报错的解法）
--------------------------------------------------------------------------------
    ① 相机直接输出 GRAYSCALE（1 B/px，RGB888 是 3 B/px）；
    ② buff_num 默认从 3 降到 2；
    ③ 算法在 WORK_SCALE（默认 0.5，即 1/4 像素量）的缩小图上做，失败才回退全分辨率；
    ④ 显示画布只分配一次并复用，不做逐帧新建；
    ⑤ 高分辨率 + MaixVision 图像预览同时开容易内存不足，必要时关掉预览。
    以 1280x720 为例：RGB888/buff3 = 8.3MB  ->  GRAYSCALE/buff2 = 1.8MB。
    仍不够时可改 /boot/configs 的 maix_memory_cmm（单位 MiB，MaixCAM2 默认 -1）。

图像形状要点（cv2.error: Bad number of channels ... 'scn' is 1 的解法）
--------------------------------------------------------------------------------
    实测 MaixCAM2 的 image.image2cv() 对 GRAYSCALE 返回的是 **(H, W, 1) 三维数组**
    （带一个哑轴），不是 (H, W)。所以绝不能按 `ndim == 2` 判"是不是灰度图"，
    否则会落到彩色分支、把单通道喂给 cv2.cvtColor(..., COLOR_BGR2GRAY) 而崩。
    本文件的 to_gray() 按**实际 shape** 判断，并兼容 (H,W)/(H,W,1)/(1,H,W)/(H,1,W)
    以及 3/4 通道；非常规打包格式（如 RGB565）交给 MaixPy 自己转灰度兜底。
"""

import os
import sys
import time


# -----------------------------------------------------------------------------
# 找到工程根目录，让"任何启动方式"都能 import 到 config / vision
# -----------------------------------------------------------------------------
# 三种文件落地位置：
#   · MaixVision「运行项目」  -> /tmp/maixpy_run/
#   · 安装成应用后离线运行    -> /root/apps/<app_id>/
#   · 手动 scp / 终端启动     -> 任意目录（甚至 cwd 与脚本不同）
# 优先用 __file__ 所在目录；个别固件里 __file__/__cwd__ 不可靠，再逐个兜底探测。
def _find_project_root():
    cands = []
    try:                                    # __file__ 可能未定义
        cands.append(os.path.dirname(os.path.abspath(__file__)))
    except NameError:
        pass
    for key in ("__cwd__", "__dir__"):      # MaixPy 部分版本提供的路径变量
        try:
            cands.append(os.path.abspath(str(globals()[key])))
        except Exception:
            pass
    try:
        cands.append(os.path.abspath(os.getcwd()))
    except Exception:
        pass
    cands += ["/tmp/maixpy_run", "/root/apps"]      # 设备上固定的两个位置
    for d in cands:
        try:
            if d and os.path.isfile(os.path.join(d, "config.py")):
                return d
        except Exception:
            continue
    return cands[0] if cands else "."


_ROOT = _find_project_root()
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _missing_project_modules():
    """返回根目录下缺失的工程模块名（单文件版已把它们放进 sys.modules，故返回空）。"""
    miss = []
    for name, rel in (("config", "config.py"), ("vision", "vision")):
        if name in sys.modules:
            continue
        if not os.path.exists(os.path.join(_ROOT, rel)):
            miss.append(name)
    return miss


# 启动方式不对时，给出能直接照做的中文提示，而不是丢一个 ModuleNotFoundError
_miss = _missing_project_modules()
if _miss:
    raise SystemExit(
        "\n[致命] 设备上只有 main.py，缺少 %s —— 这次只下发了单个文件。\n"
        "  原因：点了 MaixVision 左下角的「运行当前文件」——它只发当前这一个文件，\n"
        "        不发送工程里的其它 .py（官方文档第七节明确说明）。\n"
        "  解决（任选其一）：\n"
        "    A. 点「打开文件夹」选中工程根目录 25Efork，再点左下角「运行项目」；\n"
        "    B. 改用单文件版 dist/main_single.py（所有模块已合成一个文件），\n"
        "       打开它点「运行当前文件」即可；源码改动后在电脑上跑\n"
        "       python tools/build_single.py 重新生成。\n"
        % ", ".join(_miss)
    )

import cv2                                             # noqa: E402
import numpy as np                                     # noqa: E402

from maix import app, camera, display, image           # noqa: E402

from config import Config                              # noqa: E402
from vision.detector import TargetCenterDetector       # noqa: E402
from vision.overlay import Overlay                     # noqa: E402

# 构建标识 —— 启动时打印，用来一眼确认板子跑的是不是最新代码。
# （本项目出现过"本地 main.py 被旧内容覆盖，设备仍在跑旧逻辑"的情况，
#   日志里对不上这个版本号就说明下发的是旧文件。）
BUILD_ID = "2026-10-05-r7"

# 档位里的格式名 -> MaixPy 枚举
_FMT = {
    "GRAYSCALE": image.Format.FMT_GRAYSCALE,
    "RGB565": image.Format.FMT_RGB565,
    "RGB888": image.Format.FMT_RGB888,
    "BGR888": image.Format.FMT_BGR888,
}


# =============================================================================
# 辅助
# =============================================================================
def log_memory(tag=""):
    """打印板端内存占用，便于判断"帧缓冲内存不足"到底卡在哪。"""
    try:
        from maix import sys as m_sys
        print("[mem]%s %s" % (tag, m_sys.memory_info()))
    except Exception as e:                             # 老版本没有该接口
        print("[mem]%s 查询不可用: %s" % (tag, e))


def open_camera(cfg):
    """按"配置档位 -> 更省内存的档位"顺序尝试打开相机，成功即返回。"""
    for name, w, h, fmt, buff, fps, work in Config.profile_order(cfg.PROFILE):
        try:
            cam = camera.Camera(w, h, _FMT.get(fmt, image.Format.FMT_GRAYSCALE),
                                fps=fps, buff_num=buff)
            print("[cam] 档位 %s : %dx%d %s buff_num=%d fps=%d work_scale=%.2f"
                  % (name, w, h, fmt, buff, fps, work))
            return cam, dict(name=name, w=w, h=h, fmt=fmt, buff=buff,
                             fps=fps, work=work)
        except Exception as e:
            print("[cam] 档位 %s(%dx%d %s buff=%d) 打开失败，尝试更省内存的档位: %s"
                  % (name, w, h, fmt, buff, e))
    raise RuntimeError("所有分辨率档位都无法打开相机；请检查摄像头排线与供电。")


def _as_gray(a, bgr=False):
    """
    numpy 数组 -> 单通道灰度（能零拷贝就零拷贝）；判不出单通道则返回 None。

    **按实际 shape 判断，不按"当初请求了什么格式"猜。** 实测 MaixCAM2 的
    image2cv() 对 GRAYSCALE 返回 (H, W, 1)，而不是 (H, W)；若用 `ndim == 2`
    判灰度就会漏判 → 落到彩色分支 → 把单通道喂给 COLOR_BGR2GRAY，于是抛

        cv2.error: (-15:Bad number of channels) in function 'CvtHelper'
        > 'VScn::contains(scn)' where 'scn' is 1

    这是本项目在板上踩过的坑，预检脚本里有一项"灰度形状矩阵"专门守着它。
    """
    if a is None:
        return None
    if a.ndim == 3:
        c = a.shape[2]
        if c == 1:                                  # (H,W,1) 哑轴 —— 取视图，零拷贝
            return a[:, :, 0]
        if c == 3:                                  # RGB888 / BGR888
            return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY if bgr else cv2.COLOR_RGB2GRAY)
        if c == 4:                                  # RGBA8888 / BGRA8888
            return cv2.cvtColor(a, cv2.COLOR_BGRA2GRAY if bgr else cv2.COLOR_RGBA2GRAY)
    g = np.squeeze(a)                               # (1,H,W) / (H,1,W) 等哑轴
    return g if g.ndim == 2 else None


def to_gray(img, debug=False):
    """
    maix.image.Image -> 灰度 numpy 视图（本帧内用完即弃，勿跨帧持有）。

    优先零拷贝直出（灰度相机）；万一相机给了非常规打包格式（如 RGB565），
    交由 MaixPy 自己转灰度 —— 保证任何情况下都不会把单通道喂给 cvtColor。
    """
    a = image.image2cv(img, ensure_bgr=False, copy=False)
    if debug:
        try:
            print("[gray] format=%s shape=%s dtype=%s ndim=%d"
                  % (img.format(), tuple(a.shape), a.dtype, a.ndim))
        except Exception as e:
            print("[gray] 形状打印失败: %s" % e)
    g = _as_gray(a)
    if g is not None:
        return g
    # 罕见打包格式（如 RGB565）的兜底：让 MaixPy 做一次 BGR 归一化再处理。
    # 关键：真机上 ensure_bgr 对**灰度图是空操作**（灰度没有 BGR 形式，仍返回
    # 单通道），所以这里必须再过一次 _as_gray 按实际通道数判断，
    # 绝不能直接 cvtColor(..., COLOR_BGR2GRAY) —— 那正是板上崩掉的那一行。
    b = image.image2cv(img, ensure_bgr=True, copy=True)
    g = _as_gray(b, bgr=True)
    if g is None:
        raise RuntimeError("[gray] 无法把 shape=%s 的帧转成灰度"
                           % (tuple(getattr(b, "shape", ())),))
    return g


def pack_result(res):
    """⑧ 串口帧：AA 55 | valid(B) u(H) v(H) scale*100(H) | XOR 校验（小端）"""
    if res.get("valid"):
        u = int(max(-32768, min(32767, res["u"])))
        v = int(max(-32768, min(32767, res["v"])))
        s = int(max(0, min(65535, res["scale"] * 100)))
        body = (bytes([0xAA, 0x55, 1])
                + (u & 0xFFFF).to_bytes(2, "little")
                + (v & 0xFFFF).to_bytes(2, "little")
                + s.to_bytes(2, "little"))
    else:
        body = bytes([0xAA, 0x55, 0]) + bytes(6)
    chk = 0
    for b in body:
        chk ^= b
    return body + bytes([chk])


# =============================================================================
# 锁框失败时的诊断（真机调参全靠这个 —— 不然只能靠猜）
# =============================================================================
def _write_png(path, arr):
    # 压缩级别默认 1（不是 OpenCV 默认的 3）：板上写盘是"秒级"操作，
    # 低压缩级别显著更快、体积只大一点，对"看一眼二值图对不对"完全够用。
    lvl = max(0, min(9, int(getattr(Config, "DEBUG_DUMP_COMPRESS", 1))))
    ok, buf = cv2.imencode(".png", np.ascontiguousarray(arr),
                           [int(cv2.IMWRITE_PNG_COMPRESSION), lvl])
    if ok:
        buf.tofile(path)
        return True
    return False


def _maybe_small(arr, cfg):
    """按 DEBUG_DUMP_SCALE 降采样（1.0 = 原样）。写盘耗时随像素数平方级下降。"""
    s = float(getattr(cfg, "DEBUG_DUMP_SCALE", 1.0) or 1.0)
    if not (0.0 < s < 0.999):
        return arr
    h, w = arr.shape[:2]
    return cv2.resize(arr, (max(8, int(round(w * s))), max(8, int(round(h * s)))),
                      interpolation=cv2.INTER_AREA)


def debug_dump(gray, cfg, lock_th, lvl=0):
    """
    把"这一帧到底长什么样"落盘，便于离线分析（把这三个文件发出来即可定位问题）：
        frame_gray.png  相机原始灰度帧（未做任何处理）
        mask.png        当前阈值下的**原始**二值图（白=被判为"深色"的像素）
        mask_morph.png  过完形态学之后的二值图（描边实际看到的就是它）
    只覆盖同名文件，不会把设备磁盘写满。

    mask.png 与 mask_morph.png 的差别很关键：如果 mask.png 里胶带环是完整的、
    但 mask_morph.png 里环断了/消失了，那就是**形态学核太大把细环腐蚀掉了**
    （远距离的典型症状），该调 MORPH_OPEN_K / MORPH_CLOSE_K，而不是调阈值。

    ⚠ 本函数是**秒级**开销（板上编 3 张 PNG + 写 flash），调用方必须限流：
       main.py 用 DEBUG_DUMP_EVERY（间隔）+ DEBUG_DUMP_MAX（总量）双重限制。
       早期版本和"打印诊断"共用 2 秒节流、每次失败都写，在"检测在成败之间
       来回跳"的场景里会把帧率压到很低 —— 这是 0.1fps 的帮凶之一。
    """
    if not (cfg.DEBUG_DUMP and cfg.DEBUG_DIR):
        return
    try:
        os.makedirs(cfg.DEBUG_DIR, exist_ok=True)
        _write_png(os.path.join(cfg.DEBUG_DIR, "frame_gray.png"),
                   _maybe_small(gray, cfg))
        th = lock_th if lock_th is not None else cfg.BLACK_THRESH
        th = int(th)
        if th > 0:
            _, bw = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY_INV)
            _write_png(os.path.join(cfg.DEBUG_DIR, "mask.png"), _maybe_small(bw, cfg))
            ok = int(getattr(cfg, "MORPH_OPEN_K_FINE", 1) if lvl else cfg.MORPH_OPEN_K)
            ck = int(getattr(cfg, "MORPH_CLOSE_K_FINE", 3) if lvl else cfg.MORPH_CLOSE_K)
            mw = bw
            if ok >= 2:
                mw = cv2.morphologyEx(
                    mw, cv2.MORPH_OPEN,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok,) * 2))
            if ck >= 2:
                mw = cv2.morphologyEx(
                    mw, cv2.MORPH_CLOSE,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck,) * 2))
            _write_png(os.path.join(cfg.DEBUG_DIR, "mask_morph.png"),
                       _maybe_small(mw, cfg))
        print("[dump] 已保存 %s/{frame_gray,mask,mask_morph}.png（阈值=%d 形态档=%d 缩放=%.2f）"
              % (cfg.DEBUG_DIR, th, int(lvl), float(getattr(cfg, "DEBUG_DUMP_SCALE", 1.0))))
    except Exception as e:
        print("[dump] 保存失败(不影响运行): %s" % e)


def diag_line(det, gray, cfg):
    """
    打印"为什么没锁住框"的诊断。看这三件事基本就能定位：

      ① 灰度分布 p1/p5/p50/p95/p99 —— 判断是不是【曝光】问题
         例：p1=120 说明连最暗的胶带都有 120 灰，固定阈值 95 根本切不出黑，必须提高阈值
             或重新标定曝光；p99=60 说明整幅都黑，是欠曝。
      ② 黑占比（该阈值下被判为"深色"的像素比例）与锁定阈值
         0.00 → 框完全没进二值图；0.90+ → 整幅都黑了（背景也被算进来）
      ③ 每个 (阈值, 形态学档位) 上的候选结构（kind/分数/各项特征）—— 判断是不是【形状】问题
         有候选但分数低 → 看哪一项不达标；一个候选都没有 → 连轮廓都没形成

    ⚠ 这里**只能用 e.get(k) or 0.0 兜底**：早先直接 "%.2f" % e.get("fill") 时，
      只要某条诊断记录缺字段就会抛 `must be real number, not NoneType`，
      整行诊断全丢 —— 真机上就只剩"锁不住"，等于完全瞎调。现在记录由
      detector._dbg_rec() 统一构造（字段齐全），这里再加一层兜底，双保险。
    """
    def _n(rec, k, d=0.0):
        v = rec.get(k)
        return d if v is None else v

    try:
        p = np.percentile(gray, [1, 5, 50, 95, 99]).round(0).astype(int)
        th = det._th_lock
        thr = int(th) if th else int(cfg.BLACK_THRESH)
        _, bw = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)
        print("[diag] 灰度 p1/p5/p50/p95/p99=%d/%d/%d/%d/%d  锁定阈值=%s(形态档%d)  "
              "黑占比=%.3f  本帧来自=%s"
              % (p[0], p[1], p[2], p[3], p[4], th,
                 int(getattr(det, "_lvl_lock", 0)),
                 cv2.countNonZero(bw) / float(bw.size),
                 getattr(det, "_dbg_pass", "?")))
        rows = det.last_debug or []
        if not rows:
            print("[diag]   （本帧没生成任何候选记录 —— 说明连二值图都没跑出来）")
        for e in rows:
            print("[diag]   阈值%-5s 形态%-2s 候选%-3s 最优[%s 分=%.2f fill=%.2f rim=%.2f "
                  "ring=%.2f 长宽比=%.2f 对比=%.0f 几何合法=%s]%s"
                  % (e.get("th"), e.get("lvl"), e.get("n"), e.get("kind"),
                     _n(e, "score"), _n(e, "fill"), _n(e, "rim"), _n(e, "ring"),
                     _n(e, "aspect"), _n(e, "contrast"), e.get("ok"),
                     ("  " + e["skip"]) if e.get("skip") else ""))
    except Exception as e:
        print("[diag] 诊断输出失败(不影响运行): %s" % e)


# =============================================================================
# 主流程
# =============================================================================
def main():
    cfg = Config
    print("[app] 靶心识别 %s  （根目录 %s）" % (BUILD_ID, _ROOT))
    log_memory(" 启动前 ")

    cam, prof = open_camera(cfg)
    Config.WORK_SCALE = prof["work"]        # 交给 detector 使用
    cam.skip_frames(cfg.SKIP_FRAMES)

    if cfg.LOCK_AE_AWB:                     # 锁定曝光/白平衡：固定阈值稳定的前提
        for fn, arg in (("exp_mode", camera.AeMode.Manual),
                        ("exposure", cfg.EXPOSURE),
                        ("gain", cfg.GAIN)):
            try:
                getattr(cam, fn)(arg)
            except Exception as e:
                print("[cam] %s 设置失败(可忽略): %s" % (fn, e))
        try:
            cam.awb_mode(camera.AwbMode.Manual)
            cam.set_wb_gain(cfg.WB_GAIN)
        except Exception as e:
            print("[cam] 白平衡锁定失败(可忽略): %s" % e)
    for fn, flag in (("hmirror", cfg.H_MIRROR), ("vflip", cfg.V_FLIP)):
        try:
            getattr(cam, fn)(bool(flag))
        except Exception as e:
            print("[cam] %s 设置失败(可忽略): %s" % (fn, e))

    disp = display.Display() if cfg.SHOW else None
    det = TargetCenterDetector(cfg)
    ov = Overlay(cfg)

    uart = None
    if cfg.ENABLE_UART:
        try:
            from maix import uart as _uart
            uart = _uart.UART(cfg.UART_PORT, cfg.UART_BAUD)
        except Exception as e:
            print("[uart] 打开失败: %s" % e)

    log_memory(" 初始化后 ")
    print("[app] 进入主循环；按设备功能键退出。")

    fps, t_last, t_print = 0.0, time.time(), time.time()
    t_diag = 0.0
    t_dump = 0.0                    # 落盘单独计时（落盘是秒级开销，不能跟打印共用节流）
    n_dump = 0
    ms_det = 0.0                    # 算法耗时的指数滑动平均（板上定位瓶颈用）
    first_frame = True

    while not app.need_exit():
        t_now = time.time()
        img = cam.read()
        if img is None:
            continue

        if cfg.LENS_CORR:
            try:
                # 不同版本可能原地改或返回新对象，两种都兼容
                _c = img.lens_corr(strength=cfg.LENS_STRENGTH)
                if _c is not None:
                    img = _c
            except Exception as e:
                print("[cam] lens_corr 失败(可忽略): %s" % e)

        # 首帧打印一次真实形状，换板子/固件时用来确认走对了分支
        gray = to_gray(img, debug=first_frame and cfg.DEBUG_SHAPE)
        first_frame = False
        _t_det = time.time()
        res = det.process(gray, timestamp=t_now)  # ①~⑦
        if cfg.PROFILE_STAGES:
            ms_det = 0.9 * ms_det + 0.1 * (time.time() - _t_det) * 1000.0

        # 锁框失败：打印可定位问题的诊断（便宜），并把原始帧落盘（贵，单独限流+限量）
        if not res["valid"] and t_now - t_diag > cfg.DEBUG_DIAG_EVERY:
            t_diag = t_now
            diag_line(det, gray, cfg)
            dump_max = int(getattr(cfg, "DEBUG_DUMP_MAX", 3) or 0)
            if (t_now - t_dump > float(getattr(cfg, "DEBUG_DUMP_EVERY", 20.0))
                    and (dump_max <= 0 or n_dump < dump_max)):
                t_dump = t_now
                n_dump += 1
                debug_dump(gray, cfg, det._th_lock, getattr(det, "_lvl_lock", 0))

        if disp is not None and cfg.SHOW:
            frame = ov.render(gray, res, fps=fps)
            # copy=True：把画布拷进 maix Image，避免与显示的异步读取抢同一块内存
            disp.show(image.cv2image(frame, bgr=False, copy=True))

        if uart is not None:
            try:
                uart.write(pack_result(res))
            except Exception as e:
                print("[uart] 发送失败: %s" % e)

        dt = t_now - t_last
        t_last = t_now
        if dt > 1e-4:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        if cfg.PRINT_FPS and t_now - t_print > cfg.PRINT_EVERY:
            t_print = t_now
            cand = res.get("cand") or {}
            # 分段耗时 / 掩膜数：板上据此直接看出"瓶颈消没消"。
            # 掩膜数是最关键的一项 —— 稳态应该是个位数（锁定后是 1）；
            # 如果它长期等于 12/24，说明帧帧都在全扫，那才是 0.1fps 的真凶。
            extra = ""
            if cfg.PROFILE_STAGES:
                seg = res.get("ms") or {}
                extra = "  算法=%.1fms 掩膜=%s  [%s]" % (
                    ms_det, res.get("masks", "-"),
                    " ".join("%s=%.1f" % (k, v) for k, v in seg.items()) or "-")
            print("[fps] %.1f  valid=%s  u=%s v=%s  %.2f px/cm  阈值=%s/形态%s 候选=%s  %s%s"
                  % (fps, res["valid"],
                     "%.1f" % res["u"] if res["valid"] else "-",
                     "%.1f" % res["v"] if res["valid"] else "-",
                     res["scale"], res.get("th"), res.get("lvl", 0),
                     cand.get("kind", "-"), res["stage"], extra))

    try:
        cam.close()
    except Exception:
        pass
    print("[app] 已退出。")


if __name__ == "__main__":
    main()
