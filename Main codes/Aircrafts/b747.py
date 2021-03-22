# -*- coding: utf-8 -*-
"""
@authors: PIE n°7 group - ISAE Supaero - 2020/2021

Description : This script intends to provide the necessary intrinsic parameters
of the Boeing 747-100 plane.
        
----------
Boeing 747-100
----------

Source : JSBSim.


"""

import numpy as np

# CONVERSIONS
lbs2kg = 0.453592            # Pounds (lb) to kilograms (kg)
ft2m = 0.3048                # Feet (ft) to meters (m)
slug2kg = 14.5939            # Slug to kilograms (kg)
slugft2_2_kgm2 = 1.35581795  # Slug*feet^2 to kilograms*meters^2 (kg*m^2)


plane_data_dict = { # Inertia
                    'empty_mass': 162390,                                     # Aircraft mass [kg] 
                    'MTOW': 340200,                                           # Aircraft max take-off weight [kg] 
                    'MFL': 148540,                                            # Aircraft max kerosene capacity [kg] 
                    'inertia' : (np.diag([18200000, 33100000, 49700000]) + 
                                 [[0, 0, 970000],[0, 0, 0], [970000, 0, 0]])
                                 * slugft2_2_kgm2,                            # Aircraft inertia tensor [kg·m²]
                    'max_load_factor': 2.5*1.5,                               # Aircraft max load factor https://aviation.stackexchange.com/questions/46634/what-is-the-maximum-g-load-a-747-can-withstand-during-an-emergency-course-revers
                    'max_alpha_before_stall':20,                              # Aircraft stall angle 
                    
                    # Thrust
                    'static_thrust':240000,                                   # [N] Max static thrust
                    # 'min_static_thrust':,                                   # [N] 
                    # 'max_static_thrust':240000,                             # [N] 
                    
                    # Fuel consumption
                    'min_consumption_zero_throttle': 0,                       # [kg/h]
                    'max_consumption_full_throttle': 12100,                   # [kg/h] 
                    
                    # Amplitudes
                    'de_max':20,                                              # delta_elevator_max  [deg]
			        'da_max':30,                                              # delta_aileron_max   [deg]
			        'dr_max':20,                                              # delta_rudder_max    [deg]
                    
                    # Geometry
                    'Sw' : 511,                                               # Wing surface [m2]
                    'chord' : 8.3,                                            # Wing chord [m]
                    'span' : 59.7,                                            # Wing span [m]
                    
                    # forces_moments_coefficients
        			# Lift
				    'CL_0':0.21,                   # Zero angle of attack lift
    				'CL_a':4.4,                    # Effect of angle of attack                                          
    				'CL_q':6.6,			           # Effect of pitch rate
    				'CL_de':0.32,				   # Effect of elevator
    			    # Drag
    				'CD_0':0.0164,			       # Initial drag
    				'induced_drag_factor':0.054,   # For the lift-drag polar
    				'CD_de':0.43,  		    	   # Effect of elevator
    			    # Side_force
    				'CY_0':0, 				       # Initial side force coefficient
    				'CY_beta':-0.9,                # Effect of slide angle of aircraft
    				'CY_p':0, 				       # Effect of roll rate
    				'CY_r':0,  				       # Effect of yaw rate
    				'CY_dr':0.12, 				   # Effect of rudder
    			    # Pitch
    				'Cm_0':0,    				   # Initial pitching coefficient
    				'Cm_a':-1,				       # Effect of angle of attack
    				'Cm_q':-20.5, 				   # Effect of pitch rate
    				'Cm_de':-1.3, 				   # Effect of elevator
    			    # Roll
    				'Cl_0':0, 				       # Initial roll coefficient
    				'Cl_beta':-0.16, 			   # Effect of slide angle of aircraft
    				'Cl_p':-0.34,				   # Effect of roll rate
    				'Cl_r':0.13, 				   # Effect of yaw rate
    				'Cl_da':-0.013,				   # Effect of aileron
    				'Cl_dr':0.008,				   # Effect of rudder
    			    # Yaw
    				'Cn_0':0, 				       # Initial yaw coefficient
    				'Cn_beta':0.16, 			   # Effect of slide angle of aircraft
    				'Cn_p':-0.026, 				   # Effect of roll rate
    				'Cn_r':-0.280, 				   # Effect of yaw rate
    				'Cn_da':-0.0018, 			   # Effect of aileron
    				'Cn_dr':-0.1, 			       # Effect of rudder
                    }
			
