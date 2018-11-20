"""
    This file defines the classes and objects regarding the control strategy

"""

import numpy as np

from parameters import SimParameter, CtrParameter

from models import VehNetwork, Vehicle
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
