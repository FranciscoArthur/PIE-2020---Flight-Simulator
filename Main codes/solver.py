def rk4(plane_position_before_update, plane_orientation_before_update, plane_speed_before_update, plane_angular_speed_before_update, plane_intrinsic_data, dt, atmospheric_parameters_before_update, plane_current_forces, plane_current_moments, wind):
    """rk4 approximates the solution to an ODE using the RK4 method.

      Licensing:

          This code is distributed under the GNU LGPL license.

      Modified:

          22 April 2020

      Author:

          Nicolas Juarez

      Input:
        
        Position = [North_position, East_position, Altitude]                        #Initial conditions
        
        Orientation = [Yaw angle, Roll angle, Pitch angle]                          #Initial conditions
        
        Speed = [Velocity at x-axis, Velocity at y-axis, Velocity at z-axis ]       #Initial conditions
        
        Angular_Speed = [Yaw rate, Roll rate, Pitch rate]                           #Initial conditions
        
        plane_intrinsic_data = Dictionary containing plane mass and inertia
        
        dt = time pitch
        
        plane_current_forces = [f_x, f_y, f_z]
        
        plane_current_moments = [pitch_m, roll_m, yaw_m]



      Output:

         X_current: solution values (state vector) in the new step."""

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
    
    
    mass = plane_intrinsic_data["mass"]
    inertia = plane_intrinsic_data["inertia"]
    
    parameters = np.array([mass, inertia[0], inertia[1], inertia[2])
    
    t0 = 0 # time before update, could have been an input but since flght_equations is not time dependant, a 0 value is used.

    k1 = flight_equations(t0, X_before_update, forces, moments, parameters)
    k2 = flight_equations(t0 + dt / 2.0, X_before_update + dt * k1 / 2.0, forces, moments, parameters)
    k3 = flight_equations(t0 + dt / 2.0, X_before_update + dt * k2 / 2.0, forces, moments, parameters)
    k4 = flight_equations(t0 + dt, X_before_update + dt * k3, forces, moments, parameters)

    X_current = X_before_update + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    


    return X_current
