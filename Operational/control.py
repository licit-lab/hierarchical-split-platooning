"""
    This file defines the classes and objects regarding the control strategy

"""

import numpy as np

from parameters import SimParameter, CtrParameter

from typing import Dict, Iterable, Tuple


class TacticalCtrl(SimParameter):
    """
    Tactical Control Class
    """

    def __init__(self, sim_par: SimParameter, events: Dict = {})->None:
        SimParameter.__init__(self, sim_par.t_stp, sim_par.t_hor,
                              sim_par.t_sim)
        self.events = events
        self.reference = {}

    def __iter__(self)->Iterable:
        """
        Runs along all the registered events
        """
        self.run = iter(self.events.items())
        return self.run

    def __next__(self)->Tuple:
        """
        Run along all the registered events
        """
        return next(self.run)

    def register_veh_network(self, veh_net: VehNetwork):
        """
        Register the current veh_net in the controller so computations
        will remain valid for vehicles within this network.
        """
        self.veh_net = veh_net

    def anticipation_time(self, veh_net: VehNetwork):
        """
        Computes the anticipation time according to TRB 2018
        """
        for veh in VehNetwork:
            self.tantcp[veh] = self.e_rel/2 * \
                (1/self.a_max - 1 / self.a_min) + \
                (veh.u_ffs + veh.w_cgt) / \
                self.e_rel(self.tauf[veh]-self.tau0[veh])
        raise NotImplementedError

    def compute_tactical(self):
        """
        Compute the tactical information + update splits
        """
        raise NotImplementedError

    def compute_reference(self):
        """
        Compute reference signals for all events
        """
        t = self.sim_par.get_simtime_vector()

        for veh_id, event in zip(self):
            tau0 = event.get('tau0', 1)
            tauf = event.get('tauf', 2)
            tmerge = event.get('tm', 2)
            tantcp = event.get('ta', 1)
            b = tmerge - tantcp / 2  # Phase
            K = 8 * tantcp  # Compression
            r = tau0 + (tauf-tau0) / (1 + np.exp(-K*(t-b))
            self.reference[veh_id]=r

class OperationalCtr(CtrParameter, SimParameter):
    """
    Operational Control Class
    """
    pass
