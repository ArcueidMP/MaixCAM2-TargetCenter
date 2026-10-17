"""Existing real negatives; absent positive board captures are explicit SKIPs."""
import unittest
from pathlib import Path
from test_support import ROOT, load_config
import cv2
import numpy as np
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector


def read(path):
    return cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_GRAYSCALE)


class RealSceneTests(unittest.TestCase):
    def test_real_negatives(self):
        for name in ('negative_e421.png','negative_e880.png'):
            image=read(ROOT/'tools'/name)
            self.assertIsNotNone(image)
            self.assertFalse(RectangleCenterDetector(load_config()).process(image)['valid'],name)

    def test_front_view_direction_and_independent_center(self):
        image, truth=scene([[170,90],[430,65],[485,395],[110,420]])
        r=RectangleCenterDetector(load_config(FRONT_VIEW_SHORT_PX=512)).process(image)
        self.assertTrue(r['valid'])
        self.assertTrue(r['geometry_consistent'])
        self.assertLess(np.linalg.norm(r['front_view_center']-truth),1.5)
        view=r['front_view']
        self.assertLess(np.median(view[3,:]),50)
        self.assertGreater(np.median(view[view.shape[0]//2,:]),180)

    def test_board_positive(self):
        path=ROOT/'tools'/'real_board_frame.png'
        if not path.exists(): self.skipTest('real_board_frame.png not supplied; cannot claim 0.04px board accuracy')
        self.assertTrue(RectangleCenterDetector(load_config()).process(read(path))['valid'])

    def test_corner_gap_positive(self):
        path=ROOT/'tools'/'real_scene_corner_gap.png'
        if not path.exists(): self.skipTest('real_scene_corner_gap.png not supplied; real corner-gap precision unverified')
        self.assertTrue(RectangleCenterDetector(load_config()).process(read(path))['valid'])


if __name__ == '__main__': unittest.main()
