# -*- coding: utf-8 -*-
"""
Created on Wed Dec  2 12:03:00 2020

@author: Clément Gardies

FS_user_inputs
"""
from flight_simulator import Flight_Simulator_fct

### Considered plane
plane = ['c172']       

### Initial conditions : position, speed, orientation, angular speed, payload, fuelload

### Plane coordinates - in ground frame
# Position = [North_position, East_position, Altitude]
# Orientation = [Yaw angle, Roll angle, Pitch angle]
# Speed = [Velocity at x-axis, Velocity at y-axis, Velocity at z-axis ]
# Angular_Speed = [Yaw rate, Roll rate, Pitch rate]

initial_position = [0, 0, 100]     # [m]
initial_orientation = [0, 0, 0]
initial_speed = [50, 0, 0]        # [m/s]
initial_angular_speed = [0, 0, 0]

# Plane loading 
payload = 100                     # [kg]
initial_fuel_load = 100           # [kg]


### Weather conditions
# Wind expected to remain constant - represented by the wind ground speed vector
wind = [0, 0, 0]         
weather = [wind]                   # Next : add humidity/rain ?

### Integration parameters
time_of_study = 1
delta_t = 0.1
number_of_time_steps = int(time_of_study/delta_t) + 1    # DO NOT MODIFY

### Pilot commands, time-dependant
command_throttle_position = [0 for i in range(number_of_time_steps)]
command_rudder_position = [0 for i in range(number_of_time_steps)]
command_ailerons_position = [0 for i in range(number_of_time_steps)]
command_elevators_position = [0 for i in range(number_of_time_steps)]
command_air_brakes = [0 for i in range(number_of_time_steps)]
command_hygh_lift_devices = [0 for i in range(number_of_time_steps)]
command_landing_gear = [0 for i in range(number_of_time_steps)]







###############################################################################
# DO NOT MODIFY WHAT IS BELOW
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



### Call the main function Flight_Simulator_fct in order to compute the results and display them
result = Flight_Simulator_fct(plane, initial_conditions, weather, integration_parameters, pilot_inputs)
