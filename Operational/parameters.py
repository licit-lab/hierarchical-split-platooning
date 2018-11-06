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
    Space displ:        x_dsp
    Time displacement:  t_dsp

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
CPCTY = 0.8
U_FFS = 25

# Vehicles
L_CAV = 4.5
X_GAP_CAV = 1.75

L_HDV = 4.5
X_GAP_HDV = 3.5

K_X_CAV = 1 / (L_CAV + X_GAP_CAV)

W_CGT_CAV = CPCTY / (K_X_CAV - CPCTY/U_FFS)

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

# Minimum Required Parameters
Par = namedtuple("Parameter", ["l_veh", "x_gap", "cpcty"])
Sim = namedtuple("Simulation", ["t_stp", "t_hor", "t_sim"])

CAV_Par = Par(L_CAV, X_GAP_CAV, CPCTY)
HDV_Par = Par(L_CAV, X_GAP_CAV, CPCTY)

Sim_Par = Sim(T_STP, T_HOR, T_SIM)


class VehParameter:
    """
    Vehicle Parameters
    """

    def __init__(self,
                 u_ffs: float = U_FFS,
                 l_veh: float = L_CAV,
                 x_gap: float = X_GAP_CAV,
                 **kwargs):

        self.cpcty = None
        self.w_cgt = None
        self.u_ffs = u_ffs
        self.k_crt = None
        self.k_max = None
        self.x_dsp = None
        self.t_dsp = None

        self.l_veh = l_veh
        self.x_gap = x_gap
        self.t_gap = None
        self.x_hwy = None
        self.t_hwy = None

        self.v_drp = None
        self.fill_parameter(**kwargs)

    def __str__(self):
        return ("""{name}(\n cpcty= {cpcty},\n w_cgt= {w_cgt},\n u_ffs= {u_ffs},\n k_crt= {k_crt},\n k_max= {k_max},\n x_dsp= {x_dsp},\n t_dsp= {t_dsp},\n l_veh= {l_veh},\n x_gap= {x_gap},\n t_gap= {t_gap},\n x_hwy= {x_hwy},\n t_hwy= {t_hwy},\n v_drp= {v_drp},\n)""".format(name=self.__class__.__name__, **self.__dict__)
                )

    def __repr__(self):
        return ("""{name} \n cpcty= {cpcty},\n w_cgt= {w_cgt},\n u_ffs= {u_ffs},\n k_crt= {k_crt},\n k_max= {k_max},\n x_dsp= {x_dsp},\n t_dsp= {t_dsp},\n l_veh= {l_veh},\n x_gap= {x_gap},\n t_gap= {t_gap},\n x_hwy= {x_hwy},\n t_hwy= {t_hwy},\n v_drp= {v_drp},\n)""".format(name=self.__class__.__name__, **self.__dict__)
                )

    def fill_parameter(self, **kwargs):
        """
        Compute missing parameters
        """

        self.x_dsp = self.find_x_dsp()
        self.k_max = self.find_k_max()

        self.cpcty = kwargs.get("cpcty", None)
        if not self.cpcty:
            print(
                f"""Missing Parameters: \n C = {self.cpcty}\n No value for capacity provided \n Using congestion speed wave """
            )
            self.w_cgt = kwargs.get("w_cgt", None)
            print(f" w = {self.w_cgt}")

            self.cpcty = self.find_cpcty()

            self.k_crt = self.find_k_crt()
        else:
            self.k_crt = self.find_k_crt()
            self.w_cgt = self.find_w_cgt()

        self.t_dsp = self.find_t_dsp()

    def find_w_cgt(self):
        return self.cpcty / (self.k_max - self.k_crt)

    def find_k_crt(self):
        return self.cpcty / self.u_ffs

    def find_k_max(self):
        return 1 / self.x_dsp

    def find_cpcty(self):
        try:
            cpcty = self.w_cgt * self.u_ffs / \
                (self.w_cgt + self.u_ffs) * self.k_max
        except TypeError:
            print(
                f""" \n No value for wave congestion provided\n Using default value w: {W_CGT_CAV}""")
            self.w_cgt = W_CGT_CAV
            cpcty = self.w_cgt * self.u_ffs / \
                (self.w_cgt + self.u_ffs) * self.k_max
        return cpcty

    def find_t_dsp(self):
        return 1 / (self.k_max * self.w_cgt)

    def find_x_dsp(self):
        return self.l_veh + self.x_gap


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