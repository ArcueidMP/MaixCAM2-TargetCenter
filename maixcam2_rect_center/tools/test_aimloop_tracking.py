import unittest
from unittest.mock import patch
from test_support import Clock, Link, load_config
from test_performance import scene
from target_api import AimLoop


class AimLoopTests(unittest.TestCase):
    def test_real_detector_and_shared_control(self):
        clock=Clock()
        cfg=load_config(ENABLE_IMU_FEEDFORWARD=False)
        with patch('vision.stepper_link.StepperLink',Link):
            loop=AimLoop(cfg,clock=clock)
        image,_=scene([[180,70],[445,80],[475,400],[150,420]])
        r=loop.step(image)
        self.assertTrue(r['aim_valid'])
        count=len(loop.link.moves)
        self.assertGreater(count,0)
        r=loop.step(image,now=0)
        self.assertFalse(r['aim_valid'])
        self.assertEqual(count,len(loop.link.moves))
        clock.now=.5
        loop.idle()
        self.assertTrue(loop.link.stops)
        loop.close()
        self.assertTrue(loop.link.closed)
        with self.assertRaises(RuntimeError): loop.step(image)

    def test_detection_latency_is_age(self):
        clock=Clock()
        loop=AimLoop(load_config(),enable_motors=False,clock=clock)
        def track(frame,now):
            clock.now=.3
            return dict(valid=True,aim_valid=True,u=320,v=240,dx=26,dy=38)
        with patch.object(loop.tracker,'track',side_effect=track):
            r=loop.step(None)
        self.assertFalse(r.valid)
        self.assertEqual(r.stage,'stale')
        loop.close()


if __name__ == '__main__': unittest.main()
