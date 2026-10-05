"""桩：伪造一个会缓慢移动的靶纸场景，用于验证 main.py 主循环能跑通。"""
import numpy as np
import cv2

from . import image


class AeMode:
    Auto = 0
    Manual = 1


class AwbMode:
    Auto = 0
    Manual = 1


class Camera:
    def __init__(self, width=-1, height=-1, format=None, device=None, fps=-1,
                 buff_num=3, open=True, raw=False):
        self.w = 1280 if width in (-1, None) else width
        self.h = 720 if height in (-1, None) else height
        self.fmt = format
        self.buff_num = buff_num
        self.n = 0
        # 档位合法性：模拟"不支持的档位会抛异常"
        if self.w * self.h > 1920 * 1080:
            raise RuntimeError("unsupported resolution")

    # ---- 参数接口（只记录，不做事）----
    def exp_mode(self, m):
        pass

    def exposure(self, v):
        pass

    def gain(self, v):
        pass

    def awb_mode(self, m):
        pass

    def set_wb_gain(self, g):
        pass

    def hmirror(self, b):
        pass

    def vflip(self, b):
        pass

    def skip_frames(self, n):
        pass

    def close(self):
        pass

    def read(self):
        self.n += 1
        dx = (self.n % 7) * 3
        W, H = self.w, self.h
        scene = np.full((H, W, 3), 195, np.uint8)
        cv2.rectangle(scene, (0, H - 20), (W, H), (8, 8, 8), -1)      # 干扰条
        x0, y0 = int(0.30 * W) + dx, int(0.16 * H)
        x1, y1 = int(0.72 * W) + dx, int(0.86 * H)
        cv2.rectangle(scene, (x0, y0), (x1, y1), (20, 20, 20), -1)
        t = 16
        cv2.rectangle(scene, (x0 + t, y0 + t), (x1 - t, y1 - t), (215, 215, 210), -1)
        g = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)
        if self.fmt == image.Format.FMT_GRAYSCALE:
            return image.Image(g)
        return image.Image(scene)
