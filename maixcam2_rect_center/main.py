"""MaixPy v4 entry. Configuration is in config.py; runtime lives in vision/."""
import os
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import probe_imu
from vision.gimbal_calibrate import GimbalCalibrator
from vision.rate_meter import FrameRate
from vision.camera_source import to_gray
from vision.vision_worker import LatestFrameWorker
from vision.preview import Preview, TouchButtons, display_result, apply_maixvision_stream_policy
from vision.aim_control import VisionControl, send_uart
from vision.app_runtime import run

BUILD_ID = "2026-10-17-canny-kalman-v0.8.0"

# Compatibility aliases for existing PC tools and board integrations.
_FrameRate, _LatestVision, _Preview, _Touch = FrameRate, LatestFrameWorker, Preview, TouchButtons
_VisionControl, _display_result, _to_gray = VisionControl, display_result, to_gray
_send_uart, _apply_maixvision_stream_policy = send_uart, apply_maixvision_stream_policy


def main():
    return run(sys.modules[__name__], BUILD_ID)


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as exc:
        if exc.name != "maix":
            raise
        print("MaixPy SDK unavailable. PC: python -B tools/check_pc.py")
