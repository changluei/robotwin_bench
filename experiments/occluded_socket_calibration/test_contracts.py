"""Contracts for new route orchestration; physical evidence remains separate."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from envs.occluded_socket_calibration.controller import RouteController,ROUTES


class Contracts(unittest.TestCase):
    def controller(self,route):
        kin=SimpleNamespace(compensate=lambda:None)
        c=RouteController('S',kin,dict(dt=.004,socket_center=[.18,-.04,.77],
            module_half_size=[.038,.032,.008],drawer_center=[-.26,.18,.90]),lambda e:None,route=ROUTES[route])
        c.advance=lambda arm,t:None
        return c

    def observe_valid(self,c,t):
        frame=SimpleNamespace(rgb=np.zeros((8,8,3),np.uint8))
        d=SimpleNamespace(valid=True,T_world_socket=np.eye(4),camera='right')
        c.observe(t,{'head':frame},[d])

    def test_live_uses_later_evidence_but_initial_only_does_not(self):
        a=self.controller('direct_live');b=self.controller('direct_initial_only')
        self.observe_valid(a,.2);self.observe_valid(b,.2)
        self.assertIsNotNone(a.information);self.assertIsNone(b.information)

    def test_putdown_is_aborted_when_information_arrives_while_held(self):
        c=self.controller('putdown');c.rack_phase='placing'
        c.active['right']={'name':'rack_preapproach'}
        c.add('right','rack_release',grip=.045)
        self.observe_valid(c,.1)
        self.assertEqual(c.rack_phase,'held_ready')
        self.assertIsNone(c.active['right']);self.assertFalse(c.queues['right'])

    def test_installation_cannot_start_before_regrasp_lift_finishes(self):
        c=self.controller('putdown');c.started=True;c.rack_phase='observing'
        c.information=np.eye(4);c.status['right']='rack_withdraw_done'
        c.tick(6.)
        self.assertEqual(c.rack_phase,'recovering');self.assertFalse(c.install_started)
        self.assertEqual(c.queues['right'][-1]['name'],'rack_regrasp_lift')
        c.queues['right'].clear();c.status['right']='rack_regrasp_lift_done'
        c.tick(12.)
        self.assertTrue(c.install_started)


if __name__=='__main__':unittest.main()
