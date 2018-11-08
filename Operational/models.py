"""
    This file define the vehicle dynamics for two
    type of vehicles CAV/ HDV and methods so that 
    the dynamic evolution can be computed in a 
    global way. 
"""

import numpy as np
from typing import List

from parameters import VehParameter

X_0 = np.zeros([0])

# Dynamics defaut


def third_order(a_s_hwy, a_v_veh, a_e_veh,):
    """
        Updates according to 3rd order dynamics
    """
    au_s_hwy = a_s_hwy + T * a_e_veh
    au_v_veh = a_v_veh + T * a_a_veh
    au_e_veh = a_e_veh + T * a_
    au_a_veh = (1-T/tau) * a_a_veh + T/tau * a_u_veh


class Vehicle:
    """
        Single vehicle container 
    """
    n_veh = 0

    def __init__(self, veh_par: VehParameter,
                 veh_id: int = 0):
        self.__class__.n_veh += 1
        self.veh_id = veh_id
        self.state = np.array([])
        self.dynamic = 0

    # def update_veh_state(self):
    #     """
    #         Updates a v
    #     """
    #     pass

    # def update_dynamics(self):
    #     """"
    #         Update vehicle dynamics
    #     """"
    #     pass


class NetworkVeh:
    """
        Network of vehicles 
    """

    def __init__(self, l_veh_id: List):
        self.l_veh_id = l_veh_id

    def set_initial_condition(self,
                              m_s_hwy0: np.ndarray = X_0,
                              m_v_veh0: np.ndarray = X_0,
                              m_dv_vh0: np.ndarray = X_0
                              ):
        """ 
            Setup initial conditions of experiment
        """
        self.m_s_hwy = np.zeros()
        self.m_v_veh = np.zeros()
        self.m_dv_vh = np.zeros()

        self.m_s_hwy[0, :] = m_s_hwy0
        self.m_v_veh[0, :] = m_v_veh0
        self.m_dv_vh[0, :] = m_dv_vh0

    def create_vehicle(self, veh_par: VehParameter, veh_id: int):
        """
            Create a new Vehicle + Update network
        """
        pass

    def delete_vehicle(self, veh_id: int):
        """
            Delete an existing vehicle + Update network
        """
        pass

    def update_network(self):
        """ 
            Update Veh to be updated.
            - New Vehicles are added   (NaN->t)
            - Old Vehicles are deleted (t->NaN)
        """
        pass

    def create_virtual_veh(self):
        """
            Create leader's vehicle 
        """
        pass


if __name__ == "__main__":
    class Memoize:
        def __init__(self, fn):
            self.fn = fn
            self.memo = {}

        def __call__(self, *args):
            if args not in self.memo:
                self.memo[args] = self.fn(*args)
            return self.memo[args]

    @Memoize
    def fib(n):
        if n == 0:
            return 0
        elif n == 1:
            return 1
        else:
            return fib(n-1) + fib(n-2)

    print(fib(40))

    from timeit import Timer

    t1 = Timer("fib(10)", "from fibonacci import fib")
