import unittest
import test_support
import numpy as np
from vision.kalman_filter import ConstantVelocityKalman2D


class KalmanTests(unittest.TestCase):
    def test_noise_and_motion(self):
        rng = np.random.default_rng(9)
        k = ConstantVelocityKalman2D()
        noise = rng.normal(0, .9, (600, 2))
        filtered = np.array([k.update(z, i/30) for i,z in enumerate(noise)])
        self.assertLess(filtered[80:].std(), noise[80:].std()*.7)
        k.reset()
        truth = np.column_stack((np.arange(300)*2., np.arange(300)*.5))
        out = np.array([k.update(z, i/30) for i,z in enumerate(truth)])
        self.assertLess(np.linalg.norm(out[40:]-truth[40:], axis=1).max(), .8)

    def test_discontinuous_time_and_jump(self):
        k = ConstantVelocityKalman2D()
        k.update([1, 2], 1)
        for z,t in (([2,3],.5), ([10,20],2), ([200,300],2.03)):
            np.testing.assert_equal(k.update(z,t),z)
            np.testing.assert_equal(k.x[2:],0)

    def test_nonfinite_does_not_poison(self):
        k = ConstantVelocityKalman2D()
        k.update([1,2],0)
        x = k.x.copy()
        for z,t in (([np.nan,2],1),([1,2],np.inf)):
            with self.assertRaises(ValueError): k.update(z,t)
        np.testing.assert_equal(x,k.x)


if __name__ == '__main__': unittest.main()
