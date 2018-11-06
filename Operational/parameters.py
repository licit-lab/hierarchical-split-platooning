"""
    This file define a set of parameters for two type of
    vehicles CAV / HDV and functions so that the full
    set of parameters is consistent.

    Parameters:

    Capacity:           cpcty
    Congestion wave:    w_cgt
    Free flow speed:    u_ffs
    Critical density:   k_crt
    Maximum density:    k_max

    Vehicle length:     l_veh

    Space gap:          x_gap
    Time gap:           t_gap
    Space headway:      x_hwy
    Time headway:       t_hwy

    Speed drop:         v_drp

    Time step:          t_stp
    Time horizon:       t_hor
    Sample horizon:     s_hor

    Control weight i:   c_nbi
    Max control:        u_max
    Min control:        u_min
"""
from collections import namedtuple
import typing

# Default set of parameters
CPCTY = 2400 / 3600.0
U_FFS = 25

# Vehicles
L_CAV = 18
X_GAP_CAV = 5

L_HDV = 18
X_GAP_HDV = 10

# Simulation
T_STP = 0.1
T_HOR = 5
T_SIM = 60

# Control
C1 = 0.1
C2 = 1
C3 = 0.5
U_MAX = 1.5  # Max. Acceleration
U_MIN = -1.5  # Min. Acceleration

# Minimum Par
Par = namedtuple("Parameter", ["l_veh", "x_gap", "cpcty"])
Sim = namedtuple("Simulation", ["t_stp", "t_hor", "t_sim"])

CAV_Par = Par(L_CAV, X_GAP_CAV, CPCTY)
HDV_Par = Par(L_CAV, X_GAP_CAV, CPCTY)

Sim_Par = Sim(T_STP, T_HOR, T_SIM)


class VehParameter:
    """
    Vehicle Parameters
    """

    def __init__(self, cpcty: float = CPCTY,
                 u_ffs: float = U_FFS,
                 l_veh: float = L_CAV,
                 x_gap: float = X_GAP_CAV):

        self.cpcty = cpcty
        self.w_cgt = None
        self.u_ffs = u_ffs
        self.k_crt = None
        self.k_max = None

        self.l_veh = l_veh
        self.x_gap = x_gap
        self.t_gap = None
        self.x_hwy = None
        self.t_hwy = None

        self.v_drp = None
        self.fill_parameter()

    def __str__(self):
        return (f"{self.__class__.__name__}(cpcty = {self.cpcty}, w_cgt= {self.w_cgt}, u_ffs= {self.u_ffs}, k_crt= {self.k_crt}, k_max= {self.k_max},  l_veh= {self.l_veh}, x_gap= {self.x_gap}, t_gap= {self.t_gap}, x_hwy= {self.x_hwy}, t_hwy= {self.t_hwy},  v_drp= {self.v_drp}"
                )

    def __repr__(self):
        return (f"{self.__class__.__name__}(cpcty = {self.cpcty}, w_cgt= {self.w_cgt}, u_ffs= {self.u_ffs}, k_crt= {self.k_crt}, k_max= {self.k_max},  l_veh= {self.l_veh}, x_gap= {self.x_gap}, t_gap= {self.t_gap}, x_hwy= {self.x_hwy}, t_hwy= {self.t_hwy},  v_drp= {self.v_drp}"
                )

    def fill_parameter(self):
        """
        Compute missing parameters
        """
        self.x_hwy = self.l_veh + self.x_gap
        self.k_max = 1 / self.x_hwy
        self.k_crt = self.cpcty / self.u_ffs
        self.w_cgt = self.cpcty / (self.k_max - self.k_crt)


class SimParameter:
    """
    Simulation Parameters
    """

    def __init__(self, t_stp: float = T_STP,
                 t_hor: float = T_HOR,
                 t_sim: float = T_SIM):
        self.t_stp = t_stp
        self.t_hor = t_hor
        self.t_sim = t_sim
        self.s_hor = round(self.t_stp, self.t_hor)

    def __str__(self):
        return (f"{self.__class__.__name__}(t_stp = {self.t_stp}, t_hor= {self.t_hor}, t_sim = {self.t_sim})"
                )

    def __repr__(self):
        return (f"{self.__class__.__name__}({self.t_stp}, {self.t_hor}, {self.t_sim})"
                )


class CtrParameter:
    """
    Control Parameters
    """

    def __init__(self, c_nb1: float = C1,
                 c_nb2: float = C2,
                 c_nb3: float = C3,
                 u_min: float = U_MIN,
                 u_max: float = U_MAX):
        self.c_nb1 = c_nb1
        self.c_nb2 = c_nb2
        self.c_nb3 = c_nb3
        self.u_min = u_min
        self.u_max = u_max

    def __str__(self):
        return (f"{self.__class__.__name__}(c_nb1={self.c_nb1}, c_nb2={self.c_nb2}, c_nb3={self.c_nb3})"
                )

    def __repr__(self):
        return (f"{self.__class__.__name__}(c_nb1={self.c_nb1}, c_nb2={self.c_nb2}, c_nb3={self.c_nb3})"
                )
