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
    CPCTY = 2400/3600
    U_FFS = 25.0
    L_VEH = 18.0
    X_GAP = 55.80357142857143 - L_VEH
    veh_par = VehParameter(CPCTY, U_FFS, L_VEH, X_GAP)
    print(veh_par)

    l_veh_id = range(N_VEH)

    l_veh = [Vehicle(veh_par, veh_id) for veh_id in l_veh_id]
