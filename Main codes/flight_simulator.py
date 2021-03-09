from atmospheric_parameters import atmospheric_parameters_fct
import os
import importlib
import numpy as np


### Define the working directory
# import os
# main_path = 'C:\\Users\\Clément Gardies\\Desktop\\Projet 4A\\Avancée projet\\code_v1_gardies'
# os.chdir(main_path)


### Test : import atmospheric parameters 
# print(atmospheric_parameters_fct(10000))


### Ground frame - direct 
# Origin - arbitrary, fixed relative to the surface of the ocean
# xE axis - positive in the direction of North
# yE axis - positive in the direction of West
# zE axis - positive towards the sky (opposite to the center of the Earth)
# The earth is assumed to be flat


### Plane frame - direct
# Origin - airplane center of gravity
# xb axis - positive out the nose of the aircraft in the plane of symmetry of the aircraft
# zb axis - perpendicular to the xb axis, in the plane of symmetry of the aircraft, positive above the aircraft
# yb axis - perpendicular to the xb,zb-plane, positive determined by the right-hand rule (generally, positive out the left wing)


### Aero frame - direct
# Origin - airplane center of gravity
# xw axis - positive in the direction of the velocity vector of the aircraft relative to the air
# zw axis - perpendicular to the xw axis, in the plane of symmetry of the aircraft, positive above the aircraft
# yw axis - perpendicular to the xw,zw-plane, positive determined by the right hand rule (generally, positive to the left)

# To transforms a vector coordinates in a frame of reference to an other frame, use the script coordinates_transform.py


### Plane coordinates - in ground frame
# Position = [North_position, East_position, Altitude]
# Orientation = [Yaw angle, Roll angle, Pitch angle]
# Speed = [Velocity at x-axis, Velocity at y-axis, Velocity at z-axis ]
# Angular_Speed = [Yaw rate, Roll rate, Pitch rate]





##############################################################################
### USER INPUTS - TEST ###
# plane = ['c172']                   # Other possibility : b747

# # Plane coordinates in ground frame
# initial_position = [0, 0, 1800]     # [m]
# initial_orientation = [0, 0, 0]       
# initial_speed = [60, 0, 0]        # [m/s]      
# initial_angular_speed = [0, 0, 0]  

# # Plane loading   
# payload = 100                     # [kg]
# initial_fuel_load = 77           # [kg]
 
# # Weather conditions
# wind = [0, 0, 0]                   # wind expected to remain constant [m/s]
# weather = [wind]                   # Next : add humidity/rain ?

# # Integration parameters
# time_of_study = 1
# delta_t = 0.01
# number_of_time_steps = int(time_of_study/delta_t) + 1

# # Pilot commands
# command_throttle_position = np.array([0 for i in range(number_of_time_steps)])
# command_rudder_position = np.array([0 for i in range(number_of_time_steps)])
# command_ailerons_position = np.array([0 for i in range(number_of_time_steps)])
# command_elevators_position = np.array([0 for i in range(number_of_time_steps)])
# command_air_brakes = np.array([0 for i in range(number_of_time_steps)])
# command_hygh_lift_devices = np.array([0 for i in range(number_of_time_steps)])
# command_landing_gear = np.array([0 for i in range(number_of_time_steps)])


# ### Gathering the user's inputs
# initial_conditions=[]
# initial_conditions.append(initial_position)
# initial_conditions.append(initial_orientation)
# initial_conditions.append(initial_speed)
# initial_conditions.append(initial_angular_speed)
# initial_conditions.append(payload)
# initial_conditions.append(initial_fuel_load)

# integration_parameters =[]
# integration_parameters.append(time_of_study)
# integration_parameters.append(delta_t)
# integration_parameters.append(number_of_time_steps)


# pilot_inputs = np.array([command_throttle_position, command_rudder_position, command_ailerons_position, command_elevators_position, command_air_brakes, command_hygh_lift_devices, command_landing_gear])

### End of the TEST specification ###
##############################################################################
 


DEBUG=False



def Flight_Simulator_fct(plane, initial_conditions, weather, integration_parameters, pilot_inputs):
    
    
    ### Step 0 : collecting the user inputs
    # Plane
    plane_version = plane[0]                                        # Name of the version of the plane
    
    # Integration parameters
    time_of_study = integration_parameters[0]                       # [s]
    delta_t = integration_parameters[1]                             # [s]
    number_of_time_steps = integration_parameters[2]
    
    # State vector - creation
    plane_position = np.zeros((number_of_time_steps,3))             # Plane position - Ground frame
    plane_orientation = np.zeros((number_of_time_steps,3))          # Plane orientation - Ground frame
    plane_speed = np.zeros((number_of_time_steps,3))                # Ground speed - Ground frame
    plane_angular_speed = np.zeros((number_of_time_steps,3))        # Plane angular speed - Ground frame
    plane_fuel_load = [0] * number_of_time_steps
    plane_mass = [0] * number_of_time_steps
    plane_mach=[0]*number_of_time_steps
    
    
   # Initial conditions
    initial_position = np.asarray(initial_conditions[0])            # [m]
    initial_orientation_deg = np.asarray(initial_conditions[1])     # [deg]
    initial_orientation = initial_orientation_deg * np.pi / 180     # [rad]
    initial_speed = np.asarray(initial_conditions[2])               # [m]
    initial_angular_speed_deg = np.asarray(initial_conditions[3])   # [deg/s]
    initial_angular_speed = initial_angular_speed_deg * np.pi / 180 # [rad/s]
    payload = initial_conditions[4]                                 # [kg]
    initial_fuel_load = initial_conditions[5]                       # [kg]

    # Weather 
    wind = weather[0]


    # Pilot inputs - each command is a list of number_of_time_steps elements
    command_throttle_position = pilot_inputs[0]     # From 0 to 10
    command_rudder_position = pilot_inputs[1]       # From -10 to 10
    command_ailerons_position = pilot_inputs[2]     # From -10 to 10
    command_elevators_position = pilot_inputs[3]    # From -10 to 10
    command_air_brakes = pilot_inputs[4]            # 0 or 1
    command_hygh_lift_devices = pilot_inputs[5]     # 0 or 1
    command_landing_gear = pilot_inputs[6]          # 0 or 1



    ### Step 0_bis : Collecting the static plane data as a function of the plane version
    main_path = os.getcwd()
    plane_path = main_path + '\\' + 'Aircrafts'
    os.chdir(plane_path)
    os.getcwd()
    plane_module = importlib.import_module(plane_version)
    plane_intrinsic_data = plane_module.plane_data_dict
    os.chdir(main_path)
    # os.getcwd()



    ### Step 0_ter : Initializing the values

    # Atmospheric parameters at the initial altitude
    initial_altitude = int(initial_position[2])  
    initial_atmospheric_parameters = atmospheric_parameters_fct(-initial_altitude)
    plane_atmospheric_parameters = np.zeros((number_of_time_steps, 10))
    plane_atmospheric_parameters[0] = initial_atmospheric_parameters
    initial_air_density = initial_atmospheric_parameters[4]
    initial_sound_velocity = initial_atmospheric_parameters[5]

    
    # State vector, which initial values are given by the user inputs
    

    plane_position[0] = initial_position
    plane_orientation[0] = initial_orientation
    plane_speed[0] = initial_speed                   
    plane_angular_speed[0] = initial_angular_speed    
    plane_fuel_load[0] = initial_fuel_load    
    
    plane_initial_mass = payload + initial_fuel_load + plane_intrinsic_data['empty_mass']
    plane_mass[0] = plane_initial_mass
    
    # Resulting force and moment, null at the initial time step.
    plane_resulting_force = np.asarray([[0, 0, 0]] * number_of_time_steps)
    plane_resulting_moment = np.asarray([[0, 0, 0]] * number_of_time_steps)
    
    # Aerodynamic coefficients
    plane_aerodynamic_coefficients = np.asarray([[0, 0, 0, 0, 0, 0, 0]] * number_of_time_steps)

    # True relative AirSpeed - ground frame
    plane_TAS = [0] * number_of_time_steps
    plane_initial_TAS =  np.sqrt((initial_speed[0]-wind[0])**2 + (initial_speed[1]-wind[1])**2 + (initial_speed[2]-wind[2])**2)    # Check
    plane_TAS[0] = plane_initial_TAS
    
    plane_TAS_vector = [[0, 0, 0]] * number_of_time_steps
    plane_initial_TAS_vector = [initial_speed[0]-wind[0], initial_speed[1]-wind[1], initial_speed[2]-wind[2]]
    plane_TAS_vector[0] = plane_initial_TAS_vector


    
    # Euler angles - ground frame
    initial_psi_angle = initial_orientation[0];   # yaw angle    [rad]
    initial_phi_angle = initial_orientation[1];   # roll angle   [rad]
    initial_theta_angle = initial_orientation[2]; # pitch angle  [rad] 
    
    # Wind frame angles [https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)#Transformations_(Euler_angles)]
    plane_gamma_angle = [0] * number_of_time_steps    # Flight path angle 
    plane_sigma_angle = [0] * number_of_time_steps    # Heading angle
    # plane_mu_angle    = [0] * number_of_time_steps    # bank angle   

    if plane_initial_TAS_vector[1]>=0:           # sigma between 0 and pi
        initial_sigma_angle = np.arccos(plane_initial_TAS_vector[0]/ np.sqrt(plane_initial_TAS_vector[0]**2 + plane_initial_TAS_vector[1]**2)) 
    else:                                        # sigma between pi and 2*pi
        initial_sigma_angle = 2 * np.pi - np.arccos(plane_initial_TAS_vector[0]/ np.sqrt(plane_initial_TAS_vector[0]**2 + plane_initial_TAS_vector[1]**2)) # [rad] - between 0 and 2*pi excuded
    plane_sigma_angle[0] = initial_sigma_angle

    
    # Gamma = vertical flight angle
    initial_gamma_angle=np.arctan(initial_speed[2]/np.sqrt(initial_speed[0]**2+initial_speed[1]**2))
    plane_gamma_angle[0] = initial_gamma_angle
    
    # Body frame angles [https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)#Transformations_(Euler_angles)] 
    plane_alpha_angle = [0] * number_of_time_steps  # Angle of attack
    plane_beta_angle  = [0] * number_of_time_steps  # Sideslip angle
             
    plane_alpha_angle[0] = np.arctan(plane_initial_TAS_vector[2]/abs(plane_initial_TAS_vector[0]))
    plane_beta_angle[0] = np.arcsin(plane_initial_TAS_vector/plane_initial_TAS) 

    
    # Plane load factor - initial horizontal flight 
    plane_load_factor = [1] * number_of_time_steps
    plane_load_factor_vector = [[0, 0, 0]] * number_of_time_steps
    

    # Initial dynamic pressure calculation
    plane_qinf = [0] * number_of_time_steps
    plane_qinf[0] = 0.5*initial_air_density*(plane_initial_TAS**2)
    
    # Time evolution
    time = [0] * number_of_time_steps
    
    
    
    
    # Test conditions - initial flight failures
    if (plane_initial_mass > plane_intrinsic_data['MTOW']):
        raise ValueError ("The aircraft crashed - total weight > Max TakeOff Weight")
    
    if (initial_fuel_load > plane_intrinsic_data['MFL']):    # arbitrary max load factor
        raise ValueError ("The aircraft crashed - fuel load > max fuel load")
    # Add other errors ?    
    
    
    
    ### Entering the main loop on time - calculation of state vectors at each time step     
    i = 1
    
    while i < number_of_time_steps:
    
        if DEBUG:
            np.disp(' ')
            np.disp(' ')
            np.disp('i=' + str(i))
                
        ### Step 1 : gathering all the values that will be needed for this i-th time step calculations
        
        # Real time calculation of the selected time step 
        time[i] = i * delta_t
        
        # Pilot controls at the previous time step - the pilot commands are expected to take action at the next time step
        current_throttle_position = command_throttle_position[i-1]    
        current_rudder_position = command_rudder_position[i-1]     
        current_ailerons_position = command_ailerons_position[i-1]     
        current_elevators_position = command_elevators_position[i-1]    
        current_air_brakes = command_air_brakes[i-1]            
        current_hygh_lift_devices = command_hygh_lift_devices[i-1]    
        current_landing_gear = command_landing_gear[i-1]           
        current_pilot_controls = [current_throttle_position, current_rudder_position, current_ailerons_position, current_elevators_position, current_air_brakes, current_hygh_lift_devices, current_landing_gear]
        
        # State vector before update
        plane_position_before_update = plane_position[i-1]
        plane_orientation_before_update = plane_orientation[i-1]
        plane_speed_before_update = plane_speed[i-1]
        plane_TAS_vector_before_update = plane_TAS_vector[i-1]
        plane_TAS_before_update = plane_TAS[i-1]
        plane_angular_speed_before_update = plane_angular_speed[i-1]
        plane_fuel_load_before_update = plane_fuel_load[i-1]
        plane_mass_before_update = plane_mass[i-1]
        plane_speed_before_update = plane_speed[i-1]
        plane_mach_before_update = plane_mach[i-1]
        plane_qinf_before_update = plane_qinf[i-1]  
        
        # Atmospheric parameters at the plane altitude before update
        atmospheric_parameters_before_update = plane_atmospheric_parameters[i-1]
        
        
        ### Plane angles in the different frames
        # Euler angles - ground frame
        psi_angle_before_update = plane_orientation_before_update[0];   # yaw angle [rad]
        phi_angle_before_update = plane_orientation_before_update[1];   # roll angle [rad]
        theta_angle_before_update = plane_orientation_before_update[2]; # pitch angle [rad] 
        
        # Wind frame angles [https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)#Transformations_(Euler_angles)]
        gamma_angle_before_update = np.arccos(np.sqrt(plane_TAS_vector_before_update[0]**2 + plane_TAS_vector_before_update[1]**2)/plane_TAS_before_update) * np.sign(plane_TAS_vector_before_update[2])
        plane_gamma_angle[i] = gamma_angle_before_update
        
        if plane_TAS_vector_before_update[1]>=0:     # sigma between 0 and pi
            sigma_angle_before_update = np.arccos(plane_TAS_vector_before_update[0]/ np.sqrt(plane_TAS_vector_before_update[0]**2 + plane_TAS_vector_before_update[1]**2)) 
        else:                                        # sigma between pi and 2*pi
            sigma_angle_before_update = 2 * np.pi - np.arccos(plane_TAS_vector_before_update[0]/ np.sqrt(plane_TAS_vector_before_update[0]**2 + plane_TAS_vector_before_update[1]**2)) 
        plane_sigma_angle[i] = sigma_angle_before_update
        
        # mu_angle_before_update = 
        # plane_mu_angle[i] = mu_angle_before_update  
        
        # Body frame angles [https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)#Transformations_(Euler_angles)] 
        alpha_angle_before_update = np.arctan(plane_TAS_vector_before_update[2]/abs(plane_TAS_vector_before_update[0]))
        beta_angle_before_update = np.arcsin(plane_TAS_vector_before_update[1]/plane_TAS_before_update)   
        if DEBUG:
            np.disp('alpha = ' + str(alpha_angle_before_update * 180 / np.pi) + ' deg')
            np.disp('beta = ' + str(beta_angle_before_update * 180 / np.pi) + ' deg')
            np.disp('theta = ' + str(theta_angle_before_update * 180 / np.pi) + ' deg')
            np.disp('gamma = ' + str(gamma_angle_before_update * 180 / np.pi) + ' deg')
        # np.disp('phi = ' + str(phi_angle_before_update * 180 / np.pi) + ' deg')
        # np.disp('psi = ' + str(psi_angle_before_update * 180 / np.pi) + ' deg')
        # np.disp('sigma = ' + str(sigma_angle_before_update * 180 / np.pi) + ' deg')
        
        
        # Check - errors in body frame angles
        from coordinates_transformation import check_theta_phi_psi_range_fct, check_alpha_beta_range_fct
        check_alpha_beta_range_fct(alpha_angle_before_update, beta_angle_before_update)
        
        
        


        ### Step 2 - calculation of the aerodynamic coefficients at the current position
        """
    atmospheric_parameters_before_update :
    [0] : Altitude (m)
    [1] : Air temperature (Celsius)
    [2] : Air temperature (Kelvin)
    [3] : Air pressure (Pa)
    [4] : Air density (kg/m3)
    [5] : Air speed of sound (m/s)
    [6] : Local gravitational acceleration (m/s2)
    [7] : Air dynamic viscosity (kg/m/s)
    [8] : Air kinematic viscosity (m2/s)
    [9] : Air thermal conductivity (W/m/K)
    
    current_pilot_controls :
    [0] : throttle_position        # From 0 to 10
    [1] : rudder_position          # From -10 to 10
    [2] : ailerons_position        # From -10 to 10
    [3] : elevators_position       # From -10 to 10
    [4] : air_brakes               # 0 or 1
    [5] : hygh_lift_devices        # 0 or 1
    [6] : landing_gear             # 0 or 1
    
    OUTPUTS : current_aerodynamic_coeff :
    [CL, CD, CY, Cl, Cm, Cn, thrust]
    
        """
        from plane_data import plane_data_fct
        current_aerodynamic_coeff = plane_data_fct(plane_position_before_update, 
                                                   plane_orientation_before_update, 
                                                   plane_speed_before_update, 
                                                   plane_angular_speed_before_update,  
                                                   atmospheric_parameters_before_update,
                                                   plane_intrinsic_data,
                                                   wind,
                                                   current_pilot_controls,
                                                   plane_TAS_before_update,
                                                   plane_TAS_vector_before_update,
                                                   alpha_angle_before_update,
                                                   beta_angle_before_update,
                                                   plane_mach_before_update)

        plane_aerodynamic_coefficients[i] = current_aerodynamic_coeff
        if DEBUG:
            np.disp(' ')        
            np.disp('aero_coeff=' + str(current_aerodynamic_coeff))
        
        
              
        ### Step 3 - calculation of forces and moments    
        from forces_and_moments import forces_calculation_fct
        from forces_and_moments import moments_calculation_fct
        from forces_and_moments import fuel_consumption_calculation_fct
        from forces_and_moments import loadfactor_calculation_fct

        #Import referential change horizontal to body (state vector is in horizontal frame but it is easier to get change rate in body frame)
        from coordinates_transformation import hor2bodymatrix_fct
        hor2bodymatrix=hor2bodymatrix_fct(theta_angle_before_update,phi_angle_before_update,psi_angle_before_update)
        
        # Check if there is still some fuel 
        plane_current_forces = forces_calculation_fct(plane_mass_before_update,
                                                      plane_TAS_before_update,
                                                      plane_TAS_vector_before_update,
                                                      plane_orientation_before_update,
                                                      atmospheric_parameters_before_update,
                                                      current_aerodynamic_coeff,
                                                      plane_intrinsic_data,
                                                      hor2bodymatrix)
        if DEBUG:
            np.disp(' ')
            print ('resulting_force = ',plane_current_forces)
        
        
        plane_current_moments = moments_calculation_fct(plane_mass_before_update,
                                                        plane_TAS_before_update,
                                                        plane_orientation_before_update,
                                                        atmospheric_parameters_before_update,
                                                        current_aerodynamic_coeff,
                                                        plane_intrinsic_data)
        if DEBUG:
            np.disp(' ')
            print ('moments = ',plane_current_moments)
        
        
        plane_current_load_factor_both = loadfactor_calculation_fct(plane_mass_before_update,
                                                                    plane_current_forces,
                                                                    atmospheric_parameters_before_update,
                                                                    plane_orientation_before_update)
        if DEBUG:
            print ('Load factor = ', plane_current_load_factor_both)
        
        
        plane_current_fuel_consumption = fuel_consumption_calculation_fct(current_pilot_controls[0], plane_intrinsic_data)
                
        
        
        plane_resulting_force[i] = plane_current_forces
        plane_resulting_moment[i] = plane_current_moments
        plane_load_factor_vector[i] = plane_current_load_factor_both[0]
        plane_load_factor[i] = plane_current_load_factor_both[1]
        fuel_load_variation = plane_current_fuel_consumption * delta_t
        plane_fuel_load[i] = plane_fuel_load[i-1] - fuel_load_variation
        plane_current_mass = plane_mass[i-1] - fuel_load_variation
        plane_mass[i] = plane_current_mass
        
        
        
        
        ### Step 4 - calculation of the Derivated Plane Vector + solving the current state vector with Sover
        # CHECK NOMENCLATURE WITH ARTHUR
        
        from solver import rk4
        current_state_vector = rk4(plane_position_before_update, 
                                          plane_orientation_before_update, 
                                          plane_speed_before_update, 
                                          plane_angular_speed_before_update, 
                                          plane_intrinsic_data, 
                                          delta_t, 
                                          atmospheric_parameters_before_update, 
                                          plane_current_forces, 
                                          plane_current_moments,
                                          wind,
                                          plane_current_mass)
        
        #print (current_state_vector)
        #print('\n')
        
        # Check angles in the right range
        plane_current_theta = current_state_vector[5]
        plane_current_theta_deg = plane_current_theta * 180/np.pi
        if DEBUG:
            print ('current_theta = ',plane_current_theta_deg, 'deg')
        if np.abs(plane_current_theta)>np.pi/2:
            if np.sign(plane_current_theta) == 1:
               if np.abs(plane_current_theta)< 1.5 * np.pi:
                   plane_current_theta = np.pi - plane_current_theta
               else :
                   plane_current_theta = plane_current_theta - 2*np.pi
            else:
               if np.abs(plane_current_theta)< 1.5 * np.pi:
                   plane_current_theta = - np.pi - plane_current_theta
               else :
                   plane_current_theta = plane_current_theta + 2*np.pi  
    
        
        plane_current_phi = current_state_vector[4]
        plane_current_phi_deg = plane_current_phi * 180 / np.pi
        if DEBUG:
            print ('current_phi = ',plane_current_phi_deg, 'deg')
        if np.abs(plane_current_phi)>np.pi:
            plane_current_phi = (plane_current_phi + np.pi)%(2*np.pi) - np.pi
        
            
        
        plane_current_psi = current_state_vector[3]%(2*np.pi)      # psi between 0 and 2*pi rad
        check_theta_phi_psi_range_fct(plane_current_theta, plane_current_phi, plane_current_psi)
        
        plane_position[i] = np.array([current_state_vector[0], current_state_vector[1], current_state_vector[2]]) 
        # print('plane position =', plane_position[i])
        plane_orientation[i] = np.array([plane_current_psi, plane_current_phi, plane_current_theta])
        if DEBUG:
            print('plane orientation =', plane_orientation[i])
        plane_speed[i] = np.array([current_state_vector[6], current_state_vector[7], current_state_vector[8]])
        if DEBUG:
            print('plane speed =', plane_speed[i])
        plane_angular_speed[i] = np.array([current_state_vector[9], current_state_vector[10], current_state_vector[11]])
        if DEBUG:
            print('plane angular speed =', plane_angular_speed[i])
        
        plane_current_TAS_vector = [plane_speed[i][0]+wind[0], plane_speed[i][1]+wind[1], plane_speed[i][2]+wind[2]]
        plane_current_TAS =  np.sqrt((plane_speed[i][0]+wind[0])**2 + (plane_speed[i][1]+wind[1])**2 + (plane_speed[i][2]+wind[2])**2) 
        new_altitude = plane_position[i][2]
        if DEBUG:
            np.disp('new_altitude=' + str(new_altitude))
        new_altitude_int = int(new_altitude)
        # np.disp('new_altitude_int=' + str(new_altitude_int))        
        
        current_atmospheric_parameters = atmospheric_parameters_fct(-new_altitude_int)
        plane_atmospheric_parameters[i] = current_atmospheric_parameters
        current_air_density = current_atmospheric_parameters[4]
        current_sound_velocity = current_atmospheric_parameters[5]
        plane_TAS_vector[i] = plane_current_TAS_vector        
        plane_TAS[i] = plane_current_TAS
        plane_mach[i] = plane_current_TAS / current_sound_velocity
        plane_qinf[i] = 0.5*current_air_density*(plane_current_TAS**2)

        if DEBUG:
            np.disp('TAS=' + str(plane_current_TAS))


        ### End of a time step : Check the flight failures (ex : negative altitude)
        if (plane_position[i][2] > 0) and (current_landing_gear == 0) :
            raise ValueError ("The aircraft crashed - negative altitude")

        if np.abs(plane_load_factor[i]) > plane_intrinsic_data['max_load_factor']:  
            raise ValueError ("The aircraft crashed - unsustainable load factor")
        
        # Other errors ?
        # Add décrochage !
        
        
        
        ### Increment of the i-value = next time step
        i = i+1


    ### End of the loop : gathering all the results in a dictionary    
    result = {}
    result['plane_position'] = plane_position
    result['plane_orientation'] = plane_orientation
    result['plane_speed'] = plane_speed                    # Ground speed
    result['plane_TAS_vector'] = plane_TAS_vector          # Plane speed in the relative air - vector
    result['plane_TAS'] = plane_TAS                        # True AirSpeed - module
    result['plane_angular_speed'] =   plane_angular_speed
    result['plane_fuel_load'] = plane_fuel_load
    result['plane_mass'] = plane_mass
    result['plane_resulting_force'] = plane_resulting_force
    result['plane_resulting_moment'] = plane_resulting_moment
    result['plane_Mach'] = plane_mach
    result['plane_qinf'] = plane_qinf
    result['time'] = time   
    result['pilot_commands'] = pilot_inputs
    result['plane_load_factor'] = plane_load_factor
    result['plane_load_factor_vector'] = plane_load_factor_vector 
    result['plane_intrinsic_data'] = plane_intrinsic_data
    result['plane_atmospheric_parameters'] = plane_atmospheric_parameters
    result['plane_aerodynamic_coefficients'] = plane_aerodynamic_coefficients
    result['plane_gamma_angle']=plane_gamma_angle
    result['plane_sigma_angle']=plane_sigma_angle
    result['plane_alpha_angle']=plane_alpha_angle
    result['plane_beta_angle']=plane_beta_angle
    

     
    ### Final step - display of the results
    # CHECK NOMENCLATURE WITH NICO
    from display import display_fct
    display_fct(result)

    # return plane_intrinsic_data
    return result


