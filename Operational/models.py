"""
    This file define the vehicle dynamics for two
    type of vehicles CAV/ HDV and methods so that
    the dynamic evolution can be computed in a
    global way.
"""

import numpy as np
from numpy import ndarray

from parameters import VehParameter, SimParameter
from dynamics import dynamic_2nd, dynamic_3rd, VehDynamic

from typing import NewType, List, Callable, Union, Optional
from functools import wraps

# -------------------- TYPING --------------------

vpar = NewType('Parameter', VehParameter)
spar = NewType('SimParameter', SimParameter)
vdyn = NewType('Dynamic', Callable[[
               ndarray, ndarray, ndarray, vpar, spar], ndarray])


# -------------------- VEHICLE DYNAMICS --------------------


class VehDynamic:
    """
    Vehicle dynamics

    VehDynamic(func_dyn)

    Creates a wrapper around the function func_dyn so that

    """
    @wraps(vdyn)
    def __init__(self, veh_dyn: vdyn):
        self.veh_dyn = veh_dyn

    def __call__(self, *args, **kwargs):
        def wrap_dyn(*args, **kwargs):
            self.veh_dyn(*args, **kwargs)
        return wrap_dyn


def dynamic_3rd(veh_cst: ndarray, veh_nif: ndarray, veh_ctr: ndarray,
                veh_par: VehParameter, sim_par: SimParameter) -> ndarray:
    """
    Updates according to 3rd order dynamics

    veh_cst: Current vehicle state (s,v,e,a)
    veh_nif: Neighbor vehicle information (u)
    veh_ctr: Vehicle control input (u)
    veh_par: Vehicle parameters
    sim_par: Simulation parameters

    """
    a_s_hwy, a_v_veh, a_e_veh, a_a_veh = veh_cst
    a_a_lead = veh_nif[0]
    a_u_veh = veh_ctr[0]
    t_stp = sim_par.t_stp
    acc_lag = veh_par.v_lag
    au_s_hwy = a_s_hwy + t_stp * a_e_veh
    au_v_veh = a_v_veh + t_stp * a_a_veh
    au_e_veh = a_e_veh + t_stp * (a_a_lead-a_a_veh)
    au_a_veh = (1-t_stp/acc_lag) * a_a_veh + t_stp/acc_lag * a_u_veh
    return np.array([au_s_hwy, au_v_veh, au_e_veh, au_a_veh])


def dynamic_2nd(veh_cst: ndarray, veh_nif: ndarray, veh_ctr: ndarray,
                veh_par: VehParameter, sim_par: SimParameter) -> ndarray:
    """
    Updates according to 2nd order dynamics

    veh_cst: Current vehicle state (s,v,e)
    veh_nif: Neighbor vehicle information (u)
    veh_ctr: Vehicle control input (u)
    veh_par: Vehicle parameters
    sim_par: Simulation parameters

    """
    a_s_hwy, a_v_veh, a_e_veh = veh_cst
    a_u_lead = veh_nif[0]
    a_u_veh = veh_ctr[0]
    t_stp = sim_par.t_stp
    au_s_hwy = a_s_hwy + t_stp * a_e_veh
    au_v_veh = a_v_veh + t_stp * a_u_veh
    au_e_veh = a_e_veh + t_stp * (a_u_lead - a_u_veh)
    return np.array([au_s_hwy, au_v_veh, au_e_veh])


# -------------------- VEHICLE CLASSES --------------------


class Vehicle:
    """
    Single vehicle model
    """
    n_veh = 0

    def __init__(self, veh_dyn: VehDynamic = dynamic_2nd)->None:
        self.__class__.n_veh += 1
        self.veh_dyn = VehDynamic(veh_dyn)

    def initialize_vehicle(self, init_cond: ndarray):
        """
        Define initial condition for a vehicle
        """
        return 0


# -------------------- NETWORK CLASSES --------------------


class VehNetwork:
    """
        Network of vehicles
    """

    def __init__(self, l_veh_id: List):
        self.veh_par = VehParameter()
        self.l_veh_id = l_veh_id


if __name__ == "__main__":
    print("Launched")
