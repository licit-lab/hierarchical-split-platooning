"""
    Main Simulation.
"""
from Operational.hrctrl.models import Vehicle
from Operational.hrctrl.params import VehParameter


def CreateManager(n_veh: int):
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

    # Create vehicle parameters
    veh_par = VehParameter(U_FFS, L_VEH, X_GAP, cpcty=CPCTY)
    print(veh_par)

    # Create network of vehicles
    l_veh_id = range(N_VEH)
    l_veh = [Vehicle() for veh_id in l_veh_id]

    # Create Flow Manager (Vehicle Creation)

    # Create controller

    # Register controller in the network

    # Create Simulation Controller

    # Initiate simulation
