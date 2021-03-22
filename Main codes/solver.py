# -*- coding: utf-8 -*-
"""
@authors: PIE n°7 group - ISAE Supaero - 2020/2021

Description : This script intends to provide the necessary integration function to calculate 
the current state vector, which depends on the previous state vector and the current
resulting force and moment.

"""

def rk4(plane_position_before_update, plane_orientation_before_update, plane_speed_before_update, plane_angular_speed_before_update, plane_intrinsic_data, dt, atmospheric_parameters_before_update, plane_current_forces, plane_current_moments, wind, plane_current_mass):
    """
      'rk4' approximates the solution to an ODE using the RK4 method.

      Licensing:
          This code is distributed under the GNU LGPL license.

      Modified:
          Feburary 2021

      Input:
        Position = [North_position, East_position, Altitude]                        # Previous state vector      
        Orientation = [Yaw angle, Roll angle, Pitch angle]                          # Previous state vector        
        Speed = [Velocity at x-axis, Velocity at y-axis, Velocity at z-axis ]       # Previous state vector       
        Angular_Speed = [Yaw rate, Roll rate, Pitch rate]                           # Previous state vector        
        plane_intrinsic_data = Dictionary containing plane mass and inertia        
        dt = time pitch        
        plane_current_forces = [f_x, f_y, f_z]        
        plane_current_moments = [pitch_m, roll_m, yaw_m]        
        plane_current_mass



      Output:
         X_current: solution values (state vector) in the new step.
    """

    import numpy as np
    from flight_equations import flight_equations  
    
    
    X_before_update = np.zeros(12)
    X_before_update[0] = plane_position_before_update[0]
    X_before_update[1] = plane_position_before_update[1]
    X_before_update[2] = plane_position_before_update[2]
    X_before_update[3] = plane_orientation_before_update[0]
    X_before_update[4] = plane_orientation_before_update[1]
    X_before_update[5] = plane_orientation_before_update[2]
    X_before_update[6] = plane_speed_before_update[0]
    X_before_update[7] = plane_speed_before_update[1]
    X_before_update[8] = plane_speed_before_update[2]
    X_before_update[9] = plane_angular_speed_before_update[0]
    X_before_update[10] = plane_angular_speed_before_update[1]
    X_before_update[11] = plane_angular_speed_before_update[2]
    
  
    
    forces = plane_current_forces
    moments = plane_current_moments
    
    
    mass = plane_current_mass
    inertia = plane_intrinsic_data["inertia"]
    
    parameters = [mass, inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 2], atmospheric_parameters_before_update[6]]
    
    t0 = 0 # time before update, could have been an input but since flight_equations is not time dependant, a 0 value is used.

    # Intermediary steps using the 'flight_equations.py' script : calculation of the derived state vector
    k1 = flight_equations(t0, X_before_update, forces, moments, parameters)
    k2 = flight_equations(t0 + dt / 2.0, X_before_update + dt * k1 / 2.0, forces, moments, parameters)
    k3 = flight_equations(t0 + dt / 2.0, X_before_update + dt * k2 / 2.0, forces, moments, parameters)
    k4 = flight_equations(t0 + dt, X_before_update + dt * k3, forces, moments, parameters)

    # Runge-Kutta integrator, 4th order
    X_current = X_before_update + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0   
    # X_current = X_before_update + dt * k1
    


    return X_current
