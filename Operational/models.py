"""
    This file define the vehicle dynamics for two
    type of vehicles CAV/ HDV and methods so that
    the dynamic evolution can be computed in a
    global way.
"""

import numpy as np
from numpy import ndarray
import networkx as nx

from parameters import VehParameter, SimParameter

from collections import OrderedDict

from typing import NewType, List, Callable, Union, Optional, Dict, Iterable, Tuple
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

    Creates a wrapper around the function func_dyn so that the function
    can be called directly i.e. dynamic_3rd(...), still you can query info
    regarding the dynamics such as the name.

    This class may also save states for easiness in the computation

    """
    @wraps(vdyn)
    def __init__(self, veh_dyn: vdyn)->None:
        self.veh_dyn = veh_dyn

    def __call__(self, *args, **kwargs)->Callable:
        def wrap_dyn(*args, **kwargs):
            self.veh_dyn(*args, **kwargs)
        return wrap_dyn

    def get_name(self)->str:
        return self.veh_dyn.__name__


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
        self.control = np.array(0.0)

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

    def update_control(self,)->None:
        """
        Update a control for a vehicle
        """
        raise NotImplementedError

    def update_state(self)->None:
        """
        Update the state vector for a vehicle with the existing control
        """
        raise NotImplementedError


vehtype = Union[List[Vehicle], Vehicle]

# -------------------- NETWORK CLASSES -------------------------------------


class VehNetwork(dict):
    """
    Network of vehicles

    sim_par: simulation parameter
    vehicles: list of vehicle class
    """

    def __init__(self, sim_par: SimParameter,  vehicles: vehtype)->None:
        self.sim_par = sim_par
        self.veh_number = 0
        self.veh_currentids = []
        self.vehicles = OrderedDict()
        self.append_vehicles(vehicles)
        self.graph = None

    def __iter__(self)->Iterable:
        """
        Iterator over all vehicles in the network
        """
        self.run = iter(self.vehicles.items())
        return self.run

    def __next__(self)->Tuple:
        """
        Iterate over all vehicles in the network
        """
        return next(self.run)

    def __getitem__(self, key):
        return self.vehicles[key]

    def initialize_vehicles(self, veh_init: Dict)-> None:
        """
        Initialize condition of a single or a set of vehicles
        """
        for veh_id, state0 in veh_init.items():
            self.vehicles[veh_id].initialize_condition(state0)

    def append_vehicles(self, vehicles: vehtype)->None:
        """
        Add a single or a set of vehicles to the network
        """
        if isinstance(vehicles, Vehicle):
            self.vehicles[vehicles.id] = vehicles
            self.veh_currentids.append(vehicles.id)
            self.veh_number += 1
            return
        for veh in vehicles:
            self.vehicles[veh.id] = veh
            self.veh_currentids.append(veh.id)
            self.veh_number += 1

    def pop_vehicles(self, vehicles: vehtype)->None:
        """
        Delete a single or a set of vehicles to the network
        """
        if isinstance(vehicles, Vehicle):
            del self.vehicles[vehicles.id]
            self.veh_currentids.remove(vehicles.id)
            self.veh_number -= 1
            return
        for veh in vehicles:
            del self.vehicles[veh.id]
            self.veh_currentids.remove(veh.id)
            self.veh_number -= 1

    def create_network(self)->None:
        """
        Creates a network object that allows to query leaders
        information 
        """
        self.graph = nx.DiGraph()
        for veh_id, veh in self:
            self.graph.add_node(veh)

    def register_vehicle_link(self, veh_link: Dict)->None:
        """
        Register a list of predetermined edges for a vehicle

        veh_link = {receiver: leader}
        """
        self.veh_link = veh_link

        if not self.graph:
            self.create_network()

        for veh_id, leader_id in veh_link.items():
            self.graph.add_edge(self.vehicles[veh_id],
                                self.vehicles[leader_id])

    def get_neighbor(self, vehicle: Vehicle)->Vehicle:
        """
        Return the neighbor of a vehicle
        """
        return self.graph.neighbors(vehicle)

    def launch_simulation(self)->None:
        """
        Start a simulation
        """
        # MISSING / USE CONNECTOR FOR SIMUYVIA
        for it_step, t_step in self.sim_par:
            print(it_step, t_step)
            # self.OpCtrl.compute_control(self)
            # self.apply_control()


if __name__ == "__main__":
    print(f"Running: {__file__}")
