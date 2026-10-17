import unittest
from test_support import control, observation


class ExtensionTests(unittest.TestCase):
    def test_fault_blocks_requests_and_poll_side_effects(self):
        c,l,t=control()
        c.aim.fault('test')
        l.poll=lambda now: self.fail('FAULT must block polling/re-enable')
        c.request_test.set(); c.request_search.set()
        c.accept_result(observation(),0,0)
        self.assertFalse(l.moves)

    def test_search_requests_and_cancel(self):
        c,l,t=control()
        c.request_search.set(); c.idle()
        self.assertEqual(c.search.state,'SEARCH')
        self.assertEqual(len(l.moves),1)
        c.request_test.set(); t.now=.1; c.idle()
        self.assertIsNone(c.self_test)
        self.assertIn('rejected',c.notice)
        c.request_search.set(); c.idle()
        self.assertEqual(c.search.state,'CANCELLED')
        self.assertEqual(len(l.moves),1)

    def test_write_failure_stops_both_axes(self):
        c,l,t=control()
        l.fail_move=True
        c.accept_result(observation(30,30),0,0)
        self.assertEqual(c.aim.state,'FAULT')
        self.assertEqual(len(l.moves),1)
        self.assertEqual(l.stops[-1][0],None)

    def test_invalid_vision_stops_only_once(self):
        c,l,t=control()
        c.accept_result(observation(),0,0)
        for now in (.03,.06,.09): c.accept_result(dict(valid=False,aim_valid=False),now,now)
        self.assertEqual(len(l.stops),1)


if __name__ == '__main__': unittest.main()
