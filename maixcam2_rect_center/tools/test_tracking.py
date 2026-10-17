import unittest
from unittest.mock import patch
from test_support import load_config
import numpy as np
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector
from vision.rect_geometry import homography_8x8, project_points, rectangle_corners


class TrackingTests(unittest.TestCase):
    def test_full_search_even_after_roi_exhaustion(self):
        cfg = load_config(SEARCH_EXHAUSTIVE=False, SEARCH_MAX_PROPOSALS=128)
        d = RectangleCenterDetector(cfg)
        image,_ = scene([[180,70],[445,80],[475,400],[150,420]])
        self.assertTrue(d.process(image,0)['valid'])
        original = d._search
        calls=[]
        def search(g, roi=None):
            calls.append(roi)
            if roi is not None:
                d._proposals = cfg.SEARCH_MAX_PROPOSALS
                return None
            return original(g,roi)
        with patch.object(d,'_refine',return_value=None), patch.object(d,'_search',side_effect=search):
            result=d.process(image,.03)
        self.assertEqual(len(calls),2)
        self.assertIsNone(calls[-1])
        self.assertTrue(result['valid'])

    def test_fast_translation_and_turn(self):
        d=RectangleCenterDetector(load_config())
        q=np.array([[220,130],[340,130],[340,300],[220,300]],float)
        stages=[]
        for i in range(24):
            dx = i*10 if i < 12 else (23-i)*10
            image,truth=scene(q+[dx,0])
            r=d.process(image,i/30)
            self.assertTrue(r['valid'])
            self.assertLess(np.linalg.norm([r['raw_u']-truth[0],r['raw_v']-truth[1]]),1.)
            stages.append(r['stage'])
        self.assertGreaterEqual(stages.count('track'),22)
        self.assertFalse(d.process(np.full((480,640),200,np.uint8),1)['valid'])
        self.assertTrue(d.process(scene(q)[0],1.03)['valid'])

    def test_geometry_direction_and_frame_resize(self):
        q=np.array([[30,20],[400,30],[450,350],[20,400]],float)
        h=homography_8x8(q,210,297)
        np.testing.assert_allclose(project_points(h,rectangle_corners(210,297)),q,atol=1e-8)
        d=RectangleCenterDetector(load_config())
        d.process(scene(q)[0],0)
        r=d.process(np.ones((40,60),np.uint8),.03)
        self.assertFalse(r['valid'])
        self.assertIsNone(d._prev_quad)
        self.assertIsNone(d._smooth)


if __name__ == '__main__': unittest.main()
