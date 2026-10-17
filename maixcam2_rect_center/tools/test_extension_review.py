import unittest
from test_support import Link, load_config
from vision.gimbal_calibrate import GimbalCalibrator
from vision.gimbal_self_test import GimbalSelfTest
from vision.stepper_link import StepperLink


class EdgeCaseTests(unittest.TestCase):
    def test_visual_calibration_timeout_stops(self):
        cfg=load_config(CALIBRATE_AXIS='yaw')
        l=Link(cfg)
        c=GimbalCalibrator(cfg,l,0)
        c.update(3.1,None)
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.reason,'visual_timeout')
        self.assertTrue(l.stops)

    def test_selftest_curve_no_overlap(self):
        cfg=load_config()
        l=Link(cfg)
        s=GimbalSelfTest(cfg,l,0)
        for i in range(1000):
            l.clock.now=i*.01
            s.update(l.clock.now)
            if s.done: break
        self.assertEqual(s.state,'DONE')
        self.assertEqual(len(l.moves),6)
        for axis in ('yaw','pitch'):
            self.assertEqual(sum((1 if m[1] else -1)*m[2] for m in l.moves if m[0]==axis),0)

    def test_new_commands_cannot_postpone_ack_timeout(self):
        class Port:
            def write(self,payload): return len(payload)
        s=StepperLink.__new__(StepperLink)
        s.cfg=load_config(RX_TIMEOUT_LIMIT=2)
        s._ports={'yaw':Port()}
        s._stats=dict(tx=0,rx=0,ok=1,e2=0,err=0,timeout=0)
        s._last_cmd={'yaw':0.,'pitch':0.}
        s._last_ack={'yaw':0.,'pitch':0.}
        s._pending_ack_since={'yaw':None,'pitch':None}
        s._timeout_count={'yaw':0,'pitch':0}
        s._last_status={'yaw':None,'pitch':None}
        s._e2_times=[]
        s.fault_reason=None
        for i in range(1,30):
            now=i*.05
            s.move('yaw',True,1,60,now=now)
            feedback=s._ingest([],now)
            if feedback['fault']: break
        self.assertTrue(feedback['fault'])
        self.assertLess(now,1)


if __name__ == '__main__': unittest.main()
