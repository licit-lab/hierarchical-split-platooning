"""
    This file define the vehicle dynamics for two
    type of vehicles CAV/ HDV and methods so that
    the dynamic evolution can be computed in a
    global way.
"""

import numpy as np
from numpy import ndarray

from parameter.parameters import VehParameter, SimParameter
from models.dynamics import dynamic_2nd, dynamic_3rd, VehDynamic

from typing import NewType, List, Callable, Union, Optional
from functools import wraps

# -------------------- TYPING --------------------

vpar = NewType('Parameter', VehParameter)
spar = NewType('SimParameter', SimParameter)
vdyn = NewType('Dynamic', Callable[[
               ndarray, ndarray, ndarray, vpar, spar], ndarray])


# -------------------- DEFAULT VALUES --------------------

X_0 = np.zeros([0])


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
