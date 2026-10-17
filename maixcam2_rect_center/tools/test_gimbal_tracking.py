import unittest
from test_support import control, observation, load_config
from vision.gimbal_aim import GimbalAim
from vision.stepper_link import estimate_move_seconds


class TrackingControlTests(unittest.TestCase):
    def test_completion_requires_new_capture(self):
        c,l,t=control()
        c.accept_result(observation(),0,0)
        self.assertEqual(len(l.moves),1)
        done=c.aim._busy_until['yaw']
        c.accept_result(observation(),.03,.03)
        c.accept_result(observation(),done-.01,done+.01)
        self.assertEqual(len(l.moves),1)
        c.accept_result(observation(),done+.02,done+.02)
        self.assertEqual(len(l.moves),2)

    def test_reverse_stops_before_next_motion(self):
        c,l,t=control()
        c.accept_result(observation(),0,0)
        c.accept_result(observation(-30),.03,.03)
        self.assertEqual(l.stops[-1][0],'yaw')
        self.assertEqual(len(l.moves),1)
        c.accept_result(observation(-30),.12,.12)
        self.assertEqual(len(l.moves),2)

    def test_stop_failure_faults(self):
        c,l,t=control()
        c.accept_result(observation(),0,0)
        l.fail_stop=True
        c.accept_result(observation(0),.03,.03)
        self.assertEqual(c.aim.state,'FAULT')
        self.assertTrue(any(axis=='pitch' for axis,_ in l.stops))
        count=len(l.moves)
        c.accept_result(observation(),1,1)
        self.assertEqual(len(l.moves),count)

    def test_duplicate_future_nan_and_gap(self):
        c,l,t=control()
        c.accept_result(observation(),1,1)
        for captured,now in ((1,1.1),(.9,1.2),(3,2),(float('nan'),2)):
            self.assertFalse(c.accept_result(observation(),captured,now)['aim_valid'])
        self.assertEqual(len(l.moves),1)
        a=GimbalAim(load_config())
        for now in (0,.5,1):
            self.assertIsNone(a.update(observation(2000),now)['cmds']['yaw'])
        self.assertIsNone(a.update(observation(float('inf')),1.1)['cmds']['yaw'])

    def test_axes_progress_independently(self):
        a=GimbalAim(load_config())
        r=a.update(observation(100,3),0)['cmds']
        self.assertLess(r['pitch'][2],r['yaw'][2])
        self.assertLessEqual(r['yaw'][1],80)
        with self.assertRaises(ValueError): load_config(AIM_PROFILE='typo')
        responsive=load_config(AIM_PROFILE='responsive')
        self.assertEqual(responsive.MAX_STEP_PULSES,360)
        self.assertEqual(responsive.ACC_TRACK,80)

    def test_divergence_latches_fault_after_completed_commands(self):
        c,l,t=control(DIVERGE_CONFIRM_FRAMES=3,AIM_HISTORY_MAX_GAP_S=10.)
        for now,error in enumerate((20,40,70,120)):
            c.accept_result(observation(error),now,now)
        self.assertEqual(c.aim.state,'FAULT')
        self.assertIn('diverge',c.aim.fault_reason)
        self.assertTrue(l.stops)

    def test_curve_duration(self):
        self.assertGreater(estimate_move_seconds(80,150,80),estimate_move_seconds(80,150,0))
        self.assertGreater(estimate_move_seconds(8000,150,80),estimate_move_seconds(80,150,80))
        with self.assertRaises(ValueError): estimate_move_seconds(float('nan'),60)


if __name__ == '__main__': unittest.main()
