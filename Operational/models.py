"""
    This file define the vehicle dynamics for two
    type of vehicles CAV/ HDV and methods so that
    the dynamic evolution can be computed in a
    global way.
"""

import numpy as np
from numpy import ndarray

from parameters import VehParameter, SimParameter

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


class Vehicle(SimParameter, VehParameter):
    """
    Single vehicle model

    veh_dyn: vehicle dynamics 
    veh_id: vehicle identifier
    veh_type: vehicle type 
    veh_clane: current lane
    veh_clink: current link 
    veh_cstat: current state
    veh_ccord: current coordinates (ord, abs)
    """
    n_veh = 0

    def __init__(self, sim_par: SimParameter,
                 veh_par: VehParameter,
                 veh_dyn: VehDynamic)->None:

        self.__class__.n_veh += 1
        SimParameter.__init__(self, sim_par.t_stp, sim_par.t_hor,
                              sim_par.t_sim)

        VehParameter.__init__(self, veh_par.u_ffs, veh_par.l_veh,
                              veh_par.x_gap, cpcty=veh_par.cpcty)

        self.veh_id = None
        self.veh_dyn = VehDynamic(veh_dyn)
        self.veh_type = None
        self.veh_clane = None
        self.veh_clink = None
        self.veh_cstat = None
        self.veh_ccord = None

    def initialize_condition(self, init_cond: ndarray)->None:
        """
        Define initial condition for a vehicle
        """
        self.veh_cstat = init_cond

    def evolve_step(self):
        """

        """

# -------------------- NETWORK CLASSES --------------------


class VehNetwork(SimParameter):
    """
        Network of vehicles

        sim_par: simulation parameter 

    """

    def __init__(self, sim_par: SimParameter,  l_veh_id: List[Vehicle]):
        super().__init__(sim_par.t_stp, sim_par.t_hor,
                         sim_par.t_sim)
        self.l_veh_id = l_veh_id


if __name__ == "__main__":
    print("Launched")
