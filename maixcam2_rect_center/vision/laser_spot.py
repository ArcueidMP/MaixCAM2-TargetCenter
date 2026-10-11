"""激光光斑读数：标称位置附近的小 ROI 内找最亮点，输出亚像素质心与到靶心偏差。

为什么只搜一个小 ROI：相机与激光笔刚性固连（同装在俯仰平台上），光斑在图像里的
位置近似固定 —— 2026-10-07 现场一帧实测约 (294, 202)（640x480），背景灰度 145、
峰值 177。因此不必全图找"最亮的东西"（那会把白墙高光、灯光、反光都算进去），
只要在标称位置周围 LASER_ROI_RADIUS 像素内做一次阈值比较 + 几十个像素的质心，
代价可忽略。标称值只决定"往哪儿看"，找到的真实位置仍然逐次更新。

为什么要限频：即使单次只有零点几毫秒，也不该塞进每帧关键路径。process() 内部
按 LASER_INTERVAL_S 限频（默认 0.30s ≈ 每秒 3 次），两次更新之间直接沿用上次
结果（updated=False），所以显示仍然是每帧画的 —— 画面流畅度不受影响。

两种取位方式（`LASER_MODE`），云台侧拿到的 (u,v) 语义完全相同，只是来源不同：
    "fixed"（默认、当前方案） 光斑位置写死 = (LASER_U_NOMINAL, LASER_V_NOMINAL)，
        全程不做任何图像检测；每帧唯一的运算是"黑框中心 − 该常量"的两次减法。
        之所以可行：相机与激光刚性同转 → 光斑在图像里的位置本来就是常量（需求文档 §7.1）。
    "detect"        每 LASER_INTERVAL_S 在标称位附近 ROI 实测一次（自校准，吸收漂移）。
        只在需要重新标定 U/V、或想验证"写死的值还准不准"时临时启用。

输出契约（dict）：
    found    bool    本次是否有可信光斑（未更新时间沿用的仍是它）
    updated  bool    本次是否真的重算了（False = 限频沿用 / fixed 模式）
    u, v     float   光斑位置，相机原图坐标。**调用方必须先看 found**：found=False 时
                     u/v 只是标称值的占位，绝不能当测量结果用（deviation() 已代查）。
    peak     float   ROI 最大灰度；bg 为 ROI 背景灰度（隔行子采样中位数）
    area     int     超过阈值的像素数
    reason   str     原因码：ok / fixed / no_peak / too_big / too_small / jumped / roi
    ms       float   本次计算耗时（毫秒），未更新时为 0
"""
import time

import cv2
import numpy as np

_PANEL_BG = (32, 32, 32)
PANEL_LINE = 18
PANEL_H = 2 * PANEL_LINE + 10   # 面板高度（显示画布坐标）；main 用它把框文字整体下移
_TEXT_C1 = (255, 255, 0)     # BGR 青：光斑状态行
_TEXT_C2 = (120, 255, 120)   # BGR 浅绿：偏差行（有效）
_TEXT_C3 = (120, 120, 255)   # BGR 浅红：偏差行（无靶纸）
_MARK = (0, 0, 255)          # BGR 红：光斑标记（跟激光同色）
_LINK = (0, 160, 255)        # BGR 橙：光斑->靶心连线


class LaserSpotDetector:
    """标称位置附近找最亮点，亚像素质心 + 限频 + 跳变门限。"""

    def __init__(self, config):
        self.cfg = config
        self.mode = str(getattr(config, "LASER_MODE", "detect")).strip().lower()
        self.nominal = (float(config.LASER_U_NOMINAL), float(config.LASER_V_NOMINAL))
        self.interval = float(getattr(config, "LASER_INTERVAL_S", 0.30))
        self.alpha = float(getattr(config, "LASER_SMOOTH_ALPHA", 0.0))
        self.max_jump = float(getattr(config, "LASER_MAX_JUMP_PX", 40.0))
        self.last = dict(found=False, updated=False, u=0.0, v=0.0, peak=0.0, bg=0.0,
                         area=0, reason="init", ms=0.0)
        self._t = None
        self._uv = None

    # ------------------------------------------------------------------ 内部
    def _detect(self, g):
        """在标称位附近的方形 ROI 内做一次"阈值 + 加权质心"，返回 (u,v)。

        判据链（任一不满足即判无光斑，并带 reason 便于排障）：
            峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)  → 否则 no_peak
            亮点像素数在 [LASER_MIN_PIXELS, LASER_MAX_PIXELS] 内    → 过大 too_big（挡高光）/过小 too_small
        质心用 (灰度−阈值) 加权，越亮的像素权重越大 → 亚像素级、抗噪。
        """
        cfg = self.cfg
        H, W = g.shape[:2]
        cx, cy = float(cfg.LASER_U_NOMINAL), float(cfg.LASER_V_NOMINAL)
        r = int(cfg.LASER_ROI_RADIUS)
        # ROI = 标称位 ± r，并夹取到图像内（标称值决定"往哪儿看"，不决定结果）。
        x0 = max(0, int(cx - r))
        y0 = max(0, int(cy - r))
        x1 = min(W, int(cx + r) + 1)
        y1 = min(H, int(cy + r) + 1)
        if x1 - x0 < 5 or y1 - y0 < 5:
            return dict(found=False, reason="roi", peak=0.0, bg=0.0, area=0, u=cx, v=cy)
        roi = g[y0:y1, x0:x1]
        # 背景用 1/4 子采样中位数：抗光斑/噪点污染，又不必对 16k 像素排序。
        bg = float(np.median(roi[::4, ::4]))
        peak = float(roi.max())
        thr = max(float(cfg.LASER_MIN_PEAK), bg + float(cfg.LASER_MIN_CONTRAST))
        if peak < thr:
            return dict(found=False, reason="no_peak", peak=peak, bg=bg, area=0, u=cx, v=cy)
        mask = roi >= thr
        area = int(mask.sum())
        if area > int(cfg.LASER_MAX_PIXELS):
            # 大片高光/反光，不是激光点（激光点在 165cm 处只有几个像素）。
            return dict(found=False, reason="too_big", peak=peak, bg=bg, area=area, u=cx, v=cy)
        if area < int(cfg.LASER_MIN_PIXELS):
            return dict(found=False, reason="too_small", peak=peak, bg=bg, area=area, u=cx, v=cy)
        ys, xs = np.nonzero(mask)
        w = roi[ys, xs].astype(np.float32) - thr
        total = float(w.sum())
        if total <= 0:
            return dict(found=False, reason="flat", peak=peak, bg=bg, area=area, u=cx, v=cy)
        # 加权质心：权重 = 灰度−阈值；坐标加 ROI 原点 (x0,y0) 回到全图坐标。
        return dict(found=True, reason="ok", peak=peak, bg=bg, area=area,
                    u=x0 + float((xs * w).sum() / total), v=y0 + float((ys * w).sum() / total))

    # ------------------------------------------------------------------ 对外
    @staticmethod
    def _fixed_dict(nominal):
        return dict(found=True, updated=False, u=nominal[0], v=nominal[1],
                    peak=0.0, bg=0.0, area=0, reason="fixed", ms=0.0)

    def static_state(self):
        """fixed 模式的常量结果：光斑位置写死，不做任何图像运算。

        main.py 在 fixed 模式下只在开机取一次这个 dict，之后每帧复用；每帧唯一的
        运算是 deviation() 里的两次减法（黑框中心 − 该常量）。
        """
        return self._fixed_dict(self.nominal)

    def process(self, gray, now=None):
        """每帧调用；fixed 模式恒返回标定常量，detect 模式内部按 LASER_INTERVAL_S 限频。

        限频逻辑：距上次实测不足 interval 时**直接沿用上次结果**（updated=False, ms=0），
        因此每帧调用的代价近似为零；真正重算时依次做跳变门限与平滑。
        """
        if self.mode == "fixed":
            return self.static_state()
        now = time.perf_counter() if now is None else float(now)
        if self._t is not None and (now - self._t) < self.interval:
            out = dict(self.last)                 # 沿用上次：位置不变，仅标记未更新
            out["updated"] = False
            out["ms"] = 0.0
            return out
        self._t = now
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        started = time.perf_counter()
        res = self._detect(g)
        res["ms"] = (time.perf_counter() - started) * 1000.0
        res["updated"] = True
        if res["found"] and self._uv is not None and self.max_jump > 0:
            if abs(res["u"] - self._uv[0]) > self.max_jump or abs(res["v"] - self._uv[1]) > self.max_jump:
                # 单次跳太远：判为误检（高光/干扰物），保留上次位置。
                keep = dict(res)
                keep.update(found=True, reason="jumped", u=self._uv[0], v=self._uv[1])
                self.last = keep
                return keep
        if res["found"]:
            if self._uv is not None and 0.0 < self.alpha < 1.0:
                # 位置一阶低通（仅 detect 模式）；alpha=0 时关闭平滑，直接用最新质心。
                self._uv = (self._uv[0] + self.alpha * (res["u"] - self._uv[0]),
                            self._uv[1] + self.alpha * (res["v"] - self._uv[1]))
            else:
                self._uv = (res["u"], res["v"])
            res["u"], res["v"] = self._uv
        self.last = res
        return res


def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定取自需求文档 §7.2（dx = u靶心 − u光斑），与后续串口直控云台同一套符号：
    两轴都是"误差为正发 CCW"，因此屏幕上这个符号可以直接进控制律。

    use_raw：闭环必须用这一档（True）—— 取 result["raw_u"]/["raw_v"]（未平滑）。
    result["u"]/["v"] 是经 CENTER_ALPHA 一阶低通的值，只适合显示；拿它闭环会附加
    约 28ms 滞后（需求文档 §5.2），底盘运动时直接变成瞄准误差。
    激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])   # 闭环档：未平滑中心
    else:
        tx, ty = float(result["u"]), float(result["v"])           # 显示档：CENTER_ALPHA 平滑中心
    return (tx - float(laser["u"]), ty - float(laser["v"]))       # dx=靶心−光斑, dy 同理


def draw_laser_overlay(canvas, laser, result=None, scale=1.0, use_raw=False):
    """在显示画布左上角画激光读数面板 + 光斑标记 + 到靶心的连线。

    use_raw 与 deviation() 保持一致：True 时连线终点与 dx/dy 都用未平滑的 raw 中心，
    屏幕上看到的就是控制律真正在用的那个点。

    scale：**原图坐标 → 画布坐标** 的比例（main.py 传 render_width/原图宽）。
    本函数内部所有坐标（光斑 p、连线终点 c）都要乘 scale，才能与 draw_result 画的框对齐。
    **注意**：面板本身（左上角矩形与文字）用画布固定像素坐标，不随 scale 缩放。

    返回面板占用的高度（像素，显示画布坐标），调用方据此把 draw_result 的文字
    整体下移，避免和本面板重叠。
    """
    if not laser:
        return 0
    found = bool(laser.get("found"))
    # 第 1 行：光斑位置与来源（fixed=标定常量 / 实测 peak,area / 丢失原因）。
    if laser.get("reason") == "fixed":
        line1 = "LASER (%d,%d) [fixed/calib]" % (round(laser["u"]), round(laser["v"]))
    elif found:
        line1 = "LASER (%d,%d) peak=%d area=%d" % (
            round(laser["u"]), round(laser["v"]), round(laser["peak"]), laser.get("area", 0))
    else:
        line1 = "LASER -- [%s] peak=%d bg=%d" % (
            laser.get("reason", "?"), round(laser.get("peak", 0.0)), round(laser.get("bg", 0.0)))
    # 第 2 行：光斑→靶心偏差（用同一 use_raw 口径，与闭环一致）。
    dv = deviation(laser, result, use_raw=use_raw)
    if dv is not None:
        line2 = "D-CENTER dx=%+.1f dy=%+.1f px" % (dv[0], dv[1])
        line2_color = _TEXT_C2                       # 浅绿：有效
    elif result is not None and result.get("valid"):
        line2 = "D-CENTER -- (laser lost)"
        line2_color = _TEXT_C3                       # 浅红：有靶心无光斑
    else:
        line2 = "D-CENTER -- (no black rect)"
        line2_color = _TEXT_C3                       # 浅红：无靶心
    # 面板宽度按两行文字较宽者自适应（固定画布像素，不随 scale 缩放）。
    w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0] for t in (line1, line2))
    cv2.rectangle(canvas, (2, 2), (min(canvas.shape[1] - 1, w + 14), PANEL_H - 2), _PANEL_BG, -1)
    cv2.putText(canvas, line1, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, _TEXT_C1, 1, cv2.LINE_AA)
    cv2.putText(canvas, line2, (8, 18 + PANEL_LINE), cv2.FONT_HERSHEY_SIMPLEX, .45, line2_color, 1, cv2.LINE_AA)
    if not found:
        return PANEL_H                              # 无光斑：只留面板，不画标记/连线
    # 光斑标记：原图坐标 × scale → 画布坐标。
    p = (int(round(laser["u"] * scale)), int(round(laser["v"] * scale)))
    if result is not None and result.get("valid"):
        # 连线终点 = 靶心，同样按 use_raw 选 raw 或平滑中心，再 × scale。
        if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
            c = (int(round(float(result["raw_u"]) * scale)), int(round(float(result["raw_v"]) * scale)))
        else:
            c = (int(round(float(result["u"]) * scale)), int(round(float(result["v"]) * scale)))
        cv2.line(canvas, p, c, _LINK, 1, cv2.LINE_AA)   # 橙色连线：光斑 → 靶心
    cv2.circle(canvas, p, 7, _MARK, 1, cv2.LINE_AA)     # 红色圆圈
    cv2.drawMarker(canvas, p, _MARK, cv2.MARKER_CROSS, 5, 1, cv2.LINE_AA)   # 红十字
    return PANEL_H
