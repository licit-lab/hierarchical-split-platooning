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


class Vehicle:
    """
        Single vehicle container 
    """
    n_veh = 0

    def __init__(self, veh_par: VehParameter,
                 veh_id: int = 0):
        self.__class__.n_veh += 1
        self.veh_id = veh_id
        self.a_x_veh = np.array([0])
        self.a_v_veh = np.array([0])
        self.a_dv_vh = np.array([0])
        self.a_s_hwy = np.array([0])
        self.veh_par = veh_par


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
        """ Setup initial conditions of experiment"""
        self.m_s_hwy = np.zeros()
        self.m_v_veh = np.zeros()
        self.m_dv_vh = np.zeros()

        self.m_s_hwy[0, :] = m_s_hwy0
        self.m_v_veh[0, :] = m_v_veh0
        self.m_dv_vh[0, :] = m_dv_vh0
