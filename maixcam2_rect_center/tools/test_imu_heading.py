import unittest
from test_support import load_config
from vision.imu_heading import GyroHeading, read_gyro


class ImuTests(unittest.TestCase):
    def test_official_dual_and_zero(self):
        class Device:
            def read(self): return [0,0,1,2,3,4,25]
        self.assertEqual(read_gyro(Device()),(2.,3.,4.))
        Device.read=lambda self: [0]*7
        self.assertIsNone(read_gyro(Device()))

    def test_axis_validation(self):
        for axes in (((0,0,2),(0,-1,0)),((0,0,1),(0,0,1))):
            with self.assertRaises(ValueError): GyroHeading(load_config(IMU_YAW_AXIS=axes[0],IMU_PITCH_AXIS=axes[1]))

    def test_history_gap_and_reference(self):
        h=GyroHeading(load_config())
        for i in range(101): h.update((0,0,90),i/100)
        self.assertAlmostEqual(h.sample_at(1)['yaw'],90,places=5)
        self.assertLessEqual(h.sample_at(.505)['captured_at'],.505)
        self.assertTrue(h.lock_reference(.5))
        self.assertAlmostEqual(h.correction(1)[0],-45,places=5)
        h.update((0,0,90),2)
        self.assertIsNone(h.correction(2))
        self.assertIsNone(h.sample_at(1))
        h.update((0,0,90),1)
        self.assertAlmostEqual(h.sample_at(1)['yaw'],0)

    def test_stationary_bias_and_rejection(self):
        h=GyroHeading(load_config())
        h.start_bias_calibration(0)
        for i in range(102): h.update((.1,.2,.3),i/100)
        self.assertEqual(h.bias_state,'READY')
        self.assertAlmostEqual(h.bias[2],.3)
        h.start_bias_calibration(2)
        for i in range(102): h.update((0,0,10),2+i/100)
        self.assertEqual(h.bias_state,'REJECTED')
        self.assertAlmostEqual(h.bias[2],.3)

    def test_base_mount_requires_measured_angles(self):
        h=GyroHeading(load_config(IMU_MOUNT='base'))
        h.update((0,0,0),0)
        self.assertFalse(h.lock_reference(0))
        self.assertTrue(h.lock_reference(0,gimbal_angles=(10,20)))
        self.assertIsNone(h.correction(0))
        self.assertEqual(h.correction(0,gimbal_angles=(10,20)),(0,0))


if __name__ == '__main__': unittest.main()
