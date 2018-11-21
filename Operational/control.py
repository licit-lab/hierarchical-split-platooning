"""
    This file defines the classes and objects regarding the control strategy

"""

import numpy as np
from numpy import ndarray
from scipy.linalg import block_diag

from parameters import SimParameter, CtrParameter
from models import VehNetwork, Vehicle


from typing import Dict, Iterable, Tuple

# -------------------- MISCELANEOUS ------------------------------------


def print_matrix(mat):
    for i in range(len(mat)):
        print()
        for j in mat[i]:
            print(f'{j:.2f} \t', end='')


# -------------------- SUPERVISORY CONTROL ------------------------------------


class TacticalCtrl(SimParameter, CtrParameter):
    """
    Tactical Control Class
    """

    def __init__(self, sim_par: SimParameter,
                 ctr_par: CtrParameter, events: Dict = {})->None:
        SimParameter.__init__(self, sim_par.t_stp, sim_par.t_hor,
                              sim_par.t_sim)
        CtrParameter.__init__(self, ctr_par.c_nb1,
                              ctr_par.c_nb2, ctr_par.c_nb3, ctr_par.u_min, ctr_par.u_max, ctr_par.e_rel)
        self.events = events
        self.reference = {}
        self.tau0 = {}
        self.tauf = {}
        self.tantcp = {}

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

    def build_events(self):
        """
        Based on tm, tau0, tauf, for each vehicle computes anticipation
        time and updates the dictionary
        """
        raise NotImplementedError

    def compute_reference(self):
        """
        Compute reference signals for all events as sigmoids
        """
        t = self.get_simtime_vector()
        for veh_id, event in self:
            tau0 = event.get('tau0', 1)
            tauf = event.get('tauf', 2)
            tmerge = event.get('tm', 2)
            tantcp = event.get('ta', 1)
            b = tmerge - tantcp / 2  # Phase
            K = 8 / tantcp  # Compression
            reference = tau0 + (tauf-tau0) / (1 + np.exp(-K*(t-b)))
            self.reference[veh_id] = reference


# -------------------- OPERATIONAL CONTROL ------------------------------------


class OperationalCtr(CtrParameter, SimParameter):
    """
    Operational Control Class
    """

    def __init__(self, sim_par: SimParameter,
                 ctr_par: CtrParameter)->None:
        SimParameter.__init__(self, sim_par.t_stp, sim_par.t_hor,
                              sim_par.t_sim)
        CtrParameter.__init__(self, ctr_par.c_nb1,
                              ctr_par.c_nb2, ctr_par.c_nb3, ctr_par.u_min, ctr_par.u_max, ctr_par.e_rel)

    def build_global_dynamics(self):
        """
        Based on the network topology constructs a diagonal system to
        evolve the system
        """

        def _second_order(*args)->Tuple:
            """
            Second order dynamic matrices (s,v,e)
            """
            T, = args
            A = np.array([[1, 0, T],
                          [0, 1, 0],
                          [0, 0, 1]])
            B = np.array([[0],
                          [T],
                          [-T]])
            C = np.array([[1, 0, 0],
                          [0, 1, 0]])
            D = np.array([[0],
                          [0]])
            return A, B, C, D

        def _second_order_net(Ag, Bg, veh: Vehicle)->ndarray:
            """
            Updates the B matrix in the system to 
            account for information from neighbors
            """
            for veh_neighbor in self.veh_net.get_neighbor(veh):
                row = self._pos_mat[veh_id] * 3 + 2  # e (affected state)
                col = self._pos_mat[veh_neighbor.id]  # u (by neighbor st)
                Bg[row][col] = veh.t_stp
            return Ag, Bg

        def _third_order(*args)->Tuple:
            """
            Third order dynamic matrices (s,v,e,a)
            """
            T, Ta = args
            K = T/Ta
            A = np.array([[1, 0, T, 0],
                          [0, 1, 0, T],
                          [0, 0, 1, -T],
                          [0, 0, 0, 1-K]])
            B = np.array([[0],
                          [0],
                          [0],
                          [K]])
            C = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 0, 1]])
            D = np.array([[0],
                          [0],
                          [0]])
            return A, B, C, D

        def _third_order_net(Ag, Bg, veh: Vehicle)->ndarray:
            """
            Updates the A matrix in the system to 
            account for information from neighbors
            """
            for veh_neighbor in self.veh_net.get_neighbor(veh):
                row = self._pos_mat[veh_id] * 4 + 2  # e (affected state)
                col = self._pos_mat[veh_neighbor.id] * 4 + 3  # a (by ne)
                Ag[row][col] = veh.t_stp
            return Ag, Bg

        def _get_global_matrices(veh: Vehicle)->Tuple:
            """
            Return matrices for the corresponding dynamics
            """
            T = veh.t_stp
            Ta = veh.v_lag
            d = {"dynamic_3rd": {"fmat": _third_order,
                                 "args": (T, Ta)
                                 },
                 "dynamic_2nd": {"fmat": _second_order,
                                 "args": (T,)
                                 }
                 }
            fmat = d[veh.dynamic.get_name()]["fmat"]
            args = d[veh.dynamic.get_name()]["args"]

            return fmat(*args)

        def _get_neighbor_matrices(Ag, Bg, veh: Vehicle)->Tuple:
            """
            Return neighbor matrices for the corresponding
            dynamics 
            """
            d = {"dynamic_3rd": {"fmat": _third_order_net,
                                 "args": (Ag, Bg, veh)
                                 },
                 "dynamic_2nd": {"fmat": _second_order_net,
                                 "args": (Ag, Bg, veh)
                                 }
                 }
            fmat = d[veh.dynamic.get_name()]["fmat"]
            args = d[veh.dynamic.get_name()]["args"]

            return fmat(*args)

        def _append_matrix(X, Y): return block_diag(X, Y) if X.size else Y

        Ag, Bg, Cg, Dg = (np.array([]) for _ in range(4))

        pos_mat = 0
        self._pos_mat = {}  # Index reference

        # Creation
        for veh_id, veh in self.veh_net:
            A, B, C, D = _get_global_matrices(veh)
            Ag = _append_matrix(Ag, A)
            Bg = _append_matrix(Bg, B)
            Cg = _append_matrix(Cg, C)
            Dg = _append_matrix(Dg, D)
            self._pos_mat[veh_id] = pos_mat
            pos_mat += 1

        # Neighbor update
        for veh_id, veh in self.veh_net:
            Ag, Bg = _get_neighbor_matrices(Ag, Bg, veh)

        print_matrix(Ag)
        print_matrix(Bg)
        print('Hey')

    def register_veh_network(self, veh_net: VehNetwork):
        """
        Register the current veh_net in the controller so computations
        will remain valid for vehicles within this network.
        """
        self.veh_net = veh_net


if __name__ == "__main__":
    print(f"Running: {__file__}")
