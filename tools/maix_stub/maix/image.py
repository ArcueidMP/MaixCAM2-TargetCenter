"""桩：image 模块。重点是把 image2cv 的返回形状做得和真机一致。

★ 为什么这里要造 (H,W,1)：MaixCAM2 实测 image2cv 对 GRAYSCALE 返回的是
  3 维 (H,W,1)，而不是 2 维 (H,W)。早期桩模块返回 2 维，导致 main.py 里
  "ndim==2 判灰度"的错误分支在 PC 上永远走不到，一上板就报
  cv2.error: Bad number of channels (scn is 1)。
  把 GRAY_AS_3D 改成 False 可模拟返回 2 维的固件，两种形状都要能跑。
"""
import numpy as np
import cv2

# 灰度图是否返回 3 维 (H,W,1)。True = 模拟 MaixCAM2 实测行为。
GRAY_AS_3D = True


class Format:
    FMT_INVALID = 0xFF
    FMT_GRAYSCALE = 1
    FMT_RGB565 = 2
    FMT_RGB888 = 3
    FMT_BGR888 = 4
    FMT_RGBA8888 = 5


def _is_gray(a):
    return a.ndim == 2 or (a.ndim == 3 and a.shape[2] == 1)


class Image:
    def __init__(self, arr=None, size=None, format=None):
        if arr is not None:
            self.arr = arr
        elif size is not None:
            if format == Format.FMT_GRAYSCALE:
                self.arr = np.zeros((size[1], size[0]), np.uint8)
            else:
                self.arr = np.zeros((size[1], size[0], 3), np.uint8)
        else:
            self.arr = np.zeros((10, 10), np.uint8)

    def height(self):
        return self.arr.shape[0]

    def width(self):
        return self.arr.shape[1]

    def format(self):
        a = self.arr
        if _is_gray(a):
            return Format.FMT_GRAYSCALE
        if a.ndim == 3 and a.shape[2] == 4:
            return Format.FMT_RGBA8888
        return Format.FMT_RGB888

    def lens_corr(self, strength=1.5):
        return self

    def copy(self):
        return Image(self.arr.copy())


def image2cv(img, ensure_bgr=True, copy=True):
    """模拟真机：灰度按 GRAY_AS_3D 决定返回 2 维还是 3 维。"""
    a = img.arr
    gray = _is_gray(a)
    if gray:
        if GRAY_AS_3D and a.ndim == 2:
            a = a[:, :, None]                 # (H,W) -> (H,W,1)
        elif not GRAY_AS_3D and a.ndim == 3:
            a = a[:, :, 0]                    # (H,W,1) -> (H,W)
    # 真机上 ensure_bgr 只对彩色生效，灰度图会被原样返回（不报错但也不转 BGR）
    if ensure_bgr and a.ndim == 3 and a.shape[2] in (3, 4):
        return a[..., ::-1].copy() if copy else a[..., ::-1]
    if ensure_bgr and a.ndim == 3 and a.shape[2] == 2:      # 模拟 RGB565 -> BGR888
        return cv2.cvtColor(a, cv2.COLOR_BGR5652BGR)
    return a.copy() if copy else a


def cv2image(arr, bgr=True, copy=True):
    return Image(arr.copy() if copy else arr)
