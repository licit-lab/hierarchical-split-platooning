""" 
    Robustness analysis of MPC techniques over Truck platooning strategies.
    
    This script runs an scenario for the operational layer where 8 trucks in
    a platoon split. 

    A 3rd order model is used + following options are feasible

    1. Analysis adding noise in the measurements.
    2. Model mismatch between the control and the model.
    3. Delays within the control signal. 

    Usage: 
    python platoon-closed-3rd.py
"""
from parameters import VehParameter, SimParameter
from models import VehNetwork

# Create a simulation timings

# Create a vehicle model

# Create the network of vehicles

# Initialize the network of vehicles

# Create the controller

# Create the splitting manouvers to be executed. Tactical layer.

# Initialize the controller - (references + models to be launched)

# Evolve the system

# Scenario 1: Perturb the measurements with noise

# Scenario 2: Perturb the model parameters with noise / Parameters should keep consistancy

# Scenario 3: Save on each vehicle the full buffer but remember to retrieve the info in the past. Compute ctrl with this info and then feed the system.

# Obtain Performance measurements

# In all cases: Dynamic Answer (State + Control)


# Save results
