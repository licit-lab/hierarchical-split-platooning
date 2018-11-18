"""
    This file define the vehicle dynamics for two
    type of vehicles CAV/ HDV and methods so that
    the dynamic evolution can be computed in a
    global way.
"""

import numpy as np
from numpy import ndarray

from parameters import VehParameter, SimParameter

from typing import NewType, List, Callable, Union, Optional, Dict
from functools import wraps

# -------------------- TYPING ----------------------------------------------

vpar = NewType('Parameter', VehParameter)
spar = NewType('SimParameter', SimParameter)
vdyn = NewType('Dynamic', Callable[[
               ndarray, ndarray, ndarray, vpar, spar], ndarray])

# -------------------- VEHICLE DYNAMICS ------------------------------------


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


# -------------------- VEHICLE CLASSES -------------------------------------


class Vehicle(SimParameter, VehParameter):
    """
    Single vehicle model

    dynamic: vehicle dynamics 
    id: vehicle identifier
    type: vehicle type 
    current_lane: current lane
    current_link: current link 
    current_state: current state
    current_coordiantes: current coordinates (ord, abs)
    """
    n_veh = 0

    def __init__(self, sim_par: SimParameter,
                 veh_par: VehParameter,
                 dynamic: VehDynamic, **kwargs)->None:

        self.__class__.n_veh += 1
        SimParameter.__init__(self, sim_par.t_stp, sim_par.t_hor,
                              sim_par.t_sim)

        VehParameter.__init__(self, veh_par.u_ffs, veh_par.l_veh,
                              veh_par.x_gap, cpcty=veh_par.cpcty)

        self.id = kwargs.get('id', None)
        self.dynamic = VehDynamic(dynamic)
        self.type = kwargs.get('id', None)
        self.lane = None
        self.link = None
        self.state = None
        self.coordinates = None

    def initialize_condition(self, init_cond: ndarray, **kwargs)->None:
        """
        Define initial values for a vehicle
        """
        self.id = kwargs.get('id', self.id)
        self.type = kwargs.get('type', self.type)
        self.lane = kwargs.get('lane', self.lane)
        self.link = kwargs.get('link', self.lane)
        self.state = kwargs.get('state', init_cond)
        self.coordinates = kwargs.get('coordinates', self.coordinates)
        self.veh_cstat = init_cond

    def evolve_step(self)->None:
        raise NotImplementedError


vehtype = Union[List[Vehicle], Vehicle]

# -------------------- NETWORK CLASSES -------------------------------------


class VehNetwork(SimParameter):
    """
    Network of vehicles

    sim_par: simulation parameter
    vehicles: list of vehicle class
    """

    def __init__(self, sim_par: SimParameter,  vehicles: vehtype)->None:
        super().__init__(sim_par.t_stp, sim_par.t_hor,
                         sim_par.t_sim)
        self.veh_number = 0
        self.veh_currentids = []
        self.vehicles = []
        self.append_vehicles(vehicles)

    def initialize_network(self, veh_init: Dict)-> None:
        """
        Documents
        """
        for key, value in veh_init:
            if key not in self.veh_current_ids:
                self.register_vehicle()

    def append_vehicles(self, vehicles: vehtype)->None:
        """
        Add a single or a set of vehicles to the network
        """
        if isinstance(vehicles, Vehicle):
            self.vehicles.append(vehicles)
            self.veh_currentids.append(vehicles.id)
            self.veh_number += 1
            return
        self.vehicles += vehicles
        self.veh_currentids += [v.id for v in vehicles]
        self.veh_number += len(vehicles)

    def pop_vehicles(self, vehicles: vehtype)->None:
        """
        Delete a single or a set of vehicles to the network
        """
        if isinstance(vehicles, Vehicle):
            self.vehicles.remove(vehicles)
            self.veh_currentids.remove(vehicles.id)
            self.veh_number -= 1
            return
        new_veh = [veh for veh in self.vehicles if veh not in vehicles]
        vehid2pop = [veh.id for veh in vehicles]
        new_vehid = [
            vid for vid in self.veh_currentids if vid not in vehid2pop]
        self.vehicles = new_veh
        self.veh_currentids = new_vehid
        self.veh_number -= len(vehicles)

    def find_leader(self)->None:
        raise NotImplementedError


if __name__ == "__main__":
    print("Launched")
