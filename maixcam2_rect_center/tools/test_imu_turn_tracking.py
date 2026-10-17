import unittest
from test_support import control, observation
from vision.imu_heading import GyroHeading


class TurnTrackingTests(unittest.TestCase):
    def test_ninety_degree_turn_and_reacquisition(self):
        c,l,t=control(ENABLE_IMU_FEEDFORWARD=True,YAW_PULSES_PER_DEG=36.7,PITCH_PULSES_PER_DEG=36.7)
        h=GyroHeading(c.cfg)
        c.heading=h
        for i in range(101): h.update((0,0,90),i/100)
        r=observation(20,0)
        r['captured_at']=.9
        out=c._with_heading(r,1,True)
        # 9 degrees since exposure / -0.1 degree per pixel adds +90px, not full 90deg.
        self.assertAlmostEqual(out['dx'],110,places=5)
        c.last_capture=.9
        hold=c._with_heading(None,1,False)
        self.assertEqual(c.control_source,'imu_hold')
        self.assertAlmostEqual(hold['dx'],110,places=5)
        self.assertIsNone(c._with_heading(None,2,False))
        c._with_heading(dict(observation(0),captured_at=1),1,True)
        self.assertAlmostEqual(h.correction(1)[0],0,places=6)

    def test_uncalibrated_no_degree_commands(self):
        c,l,t=control(ENABLE_IMU_FEEDFORWARD=True)
        c.heading=GyroHeading(c.cfg)
        c.heading.update((0,0,0),0)
        self.assertIsNone(c._degree_scales())


if __name__ == '__main__': unittest.main()
