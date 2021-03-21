# -*- coding: utf-8 -*-
"""
Created on Wed Dec  2 12:03:00 2020

@author: Clément Gardies

FS_user_inputs
"""
from flight_simulator import Flight_Simulator_fct
import numpy as np

### Considered plane
plane = ['c172']       



### Initial conditions : position, speed, orientation, angular speed, payload, fuelload

### Plane coordinates - in ground frame
# Position = [North_position, East_position, Altitude]
# Orientation = [Yaw angle, Roll angle, Pitch angle]
# Speed = [Velocity at x-axis, Velocity at y-axis, Velocity at z-axis]
# Angular_Speed = [Yaw rate, Roll rate, Pitch rate]

initial_position = [0, 0, 1800]     # [m]
initial_orientation = [0, 0, -0.8]     # [deg]
initial_speed = [70.3, 0, 0]          # [m/s]
initial_angular_speed = [0, 0, 0]   # [deg/s]


# Plane loading 
payload = 100                       # [kg]
initial_fuel_load = 77              # [kg]


### Weather conditions
wind = [0, 0, 0]                    # Constant wind vector - ground frame [m/s]
weather = [wind]                    # Next : add humidity/rain ?


### Integration parameters
time_of_study = 500                # [s]
delta_t = 0.01                      # [s]
number_of_time_steps = int(time_of_study/delta_t) + 1    # DO NOT MODIFY


### Pilot commands, time-dependant - each command is a list of number_of_time_steps elements

# Elevators commands
# command_elevators_position = ([-1, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0] * (int(number_of_time_steps/21) + 20))[:number_of_time_steps]
# command_elevators_position = ([-1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0] * (int(number_of_time_steps/20) + 19))[:number_of_time_steps]
# command_elevators_position = ([-1, 0, 0,  0, 0, 0,  0, -1, 0, 0,  0, 0, 0,  0, -1] * (int(number_of_time_steps/15) + 14))[:number_of_time_steps]
# command_elevators_position = np.array([-1 for i in range(number_of_time_steps)])   # From -10 to 10
command_elevators_position = np.array([0. for i in range(number_of_time_steps)])   # From -10 to 10

# Rudder commands
command_rudder_position = np.array([0. for i in range(number_of_time_steps)])      # From -10 to 10
# command_rudder_position[10] = 1
# command_rudder_position[20] = -1
# command_rudder_position[21] = -1

# Ailerons commands
command_ailerons_position = np.array([0. for i in range(number_of_time_steps)])    # From -10 to 10
# command_ailerons_position[1000:1300]=1
# command_ailerons_position[10] = 1
# command_ailerons_position[11] = -1
# command_ailerons_position[12] = -1

# Throttle commands
command_throttle_position = np.array([7. for i in range(number_of_time_steps)])    # From 0 to 10

command_air_brakes = np.array([0 for i in range(number_of_time_steps)])           # 0 or 1
command_hygh_lift_devices = np.array([0 for i in range(number_of_time_steps)])    # 0 or 1
command_landing_gear = np.array([0 for i in range(number_of_time_steps)])         # 0 or 1



# Optional control order
# Define a targeted altitude or leave 'False' for uncontrolled flight
# If a targeted altitude is defined, it must be equal to the initial altitude defined above

#Target_altitude = False
Target_altitude = 1800







###############################################################################
# DO NOT MODIFY WHAT IS BELOW

#Positive to minus altitude
initial_position[2]*=-1

### Gathering the user's inputs
initial_conditions=[]
initial_conditions.append(initial_position)
initial_conditions.append(initial_orientation)
initial_conditions.append(initial_speed)
initial_conditions.append(initial_angular_speed)
initial_conditions.append(payload)
initial_conditions.append(initial_fuel_load)

integration_parameters =[]
integration_parameters.append(time_of_study)
integration_parameters.append(delta_t)
integration_parameters.append(number_of_time_steps)

pilot_inputs = [command_throttle_position, command_rudder_position, command_ailerons_position, command_elevators_position, command_air_brakes, command_hygh_lift_devices, command_landing_gear]
# print(pilot_inputs)


### Call the main function Flight_Simulator_fct in order to compute the results and display them
result = Flight_Simulator_fct(plane, initial_conditions, weather, integration_parameters, pilot_inputs, Target_altitude)
