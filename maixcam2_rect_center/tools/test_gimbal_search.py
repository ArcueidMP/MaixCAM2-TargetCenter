import unittest
from test_support import Link, load_config
from vision.gimbal_search import SearchSweep


class SearchTests(unittest.TestCase):
    def test_uncalibrated_fallback_and_cancel(self):
        link=Link()
        search=SearchSweep(load_config(),link,0)
        search.update(0)
        self.assertEqual(link.moves[0][2],400)
        self.assertEqual(len(search.sequence),6)
        search.cancel(.1)
        count=len(link.moves)
        search.update(1)
        self.assertEqual(len(link.moves),count)
        self.assertEqual(search.state,'CANCELLED')

    def test_freshness_and_fault(self):
        link=Link()
        search=SearchSweep(load_config(),link,2)
        search.update(2,dict(valid=True,captured_at=1.99))
        self.assertEqual(search.state,'SEARCH')
        search.update(2.1,dict(valid=True,captured_at=2.1,predicted=True))
        self.assertEqual(search.state,'SEARCH')
        search.update(2.2,dict(valid=True,captured_at=2.2))
        self.assertEqual(search.state,'FOUND')
        link.fail_stop=True
        s=SearchSweep(load_config(),link,3)
        s.cancel(3)
        self.assertEqual(s.state,'FAULT')

    def test_imu_early_stop(self):
        class Heading:
            def sample_at(self,now): return dict(yaw=now*100,pitch=0,epoch=1)
        link=Link()
        s=SearchSweep(load_config(),link,0,Heading())
        s.update(0)
        s.update(.43)
        self.assertEqual(link.stops[-1][0],'yaw')
        self.assertTrue(s.measurements[0]['upper_bound'])


if __name__ == '__main__': unittest.main()
