"""
    Main Simulation.
"""

from parameters import VehParameter
from models import Vehicle, NetworkVeh


def CreateVeh(n_veh: int):
    return n_veh


if __name__ == "__main__":

    # number of vehicles
    N_VEH = 8

    # Parameters
    CPCTY = 0.8
    U_FFS = 25.0

    # Fixed for a K_MAX = 0.16
    L_VEH = 4.5
    X_GAP = 1.75

    veh_par = VehParameter(U_FFS, L_VEH, X_GAP, cpcty=CPCTY)
    print(veh_par)

    l_veh_id = range(N_VEH)

    l_veh = [Vehicle(veh_par, veh_id) for veh_id in l_veh_id]
