# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 17:37:32 2021

----------
Cessna 172
----------

Source : JSBSim.


"""

import numpy as np

# CONVERSIONS
lbs2kg = 0.453592  # Pounds (lb) to kilograms (kg)
ft2m = 0.3048  # Feet (ft) to meters (m)
slug2kg = 14.5939  # Slug to kilograms (kg)
slugft2_2_kgm2 = 1.35581795  # Slug*feet^2 to kilograms*meters^2 (kg*m^2)


plane_data_dict = { # Inertia
                    'empty_mass': 780,                                        # Aircraft mass [kg] 
                    'MTOW': 1160,                                             # Aircraft max take-off weight [kg] 
                    'MFL': 163,                                               # Aircraft max kerosene capacity [kg]                     
                    'inertia' : np.diag([948, 1346, 1967]) * slugft2_2_kgm2,  # Aircraft inertia tensor [kg·m²]
 
                    # Thrust
                    'static_thrust':1000,                                     # [N] 
                    # 'min_static_thrust':,                                     # [N] 
                    # 'max_static_thrust':1000,                                     # [N] 
                    
                    # Fuel consumption
                    'min_consumption_zero_throttle': 6.4,                     # [kg/h]
                    'max_additional_consumption_full_throttle': 1.85,         # [kg/h] 
                    
                    # Amplitudes
                    'de_max':25,                                              # delta_elevator_max  [deg]
			        'da_max':20,                                              # delta_aileron_max   [deg]
			        'dr_max':16,                                              # delta_rudder_max    [deg]
                    
                    # Geometry
                    'Sw' : 16.2,                                              # Wing surface [m2]
                    'chord' : 1.49352,                                        # Wing chord [m]
                    'span' : 10.91184,                                        # Wing span [m]
                    
                    # forces_moments_coefficients
        			# Lift
				    'CL_0':0.31,			#Zero angle of attack lift
    				'CL_a':5.143,                           #Effect of angle of attack
    				'CL_q':3.9,				#Effect of roll rate
    				'CL_de':0.43,				#Effect of elevator
    			    # Drag
    				'CD_0':0.031,				#Initial drag
    				'induced_drag_factor':0.054, 		#For the lift-drag polar
    				'CD_de':0.43,				#Effect of elevator
    			    # Side_force
    				'CY_0':0, 				#Initial side force coefficient
    				'CY_beta':-0.31, 			#Effect of slide angle of aircraft
    				'CY_p':-0.037, 				#Effect of pitch rate
    				'CY_r':0.21, 				#Effect of yaw rate
    				'CY_dr':0.187, 				#Effect of rudder
    			    # Pitch
    				'Cm_0':-0.015, 				#Initial pitching moment
    				'Cm_da':-0.89,				#Effect of aileron
    				'Cm_q':-12.4, 				#Effect of roll rate
    				'Cm_de':-1.28, 				#Effect of elevator
    			    # Roll
    				'Cl_0':0, 				#Initial roll coefficient
    				'Cl_beta':-0.089, 			#Effect of slide angle of aircraft
    				'Cl_p':-0.47,				#Effect of pitch rate
    				'Cl_r':0.096, 				#Effect of yaw rate
    				'Cl_da':-0.178,				#Effect of aileron
    				'Cl_dr':0.0147,				#Effect of rudder
    			    # Yaw
    				'Cn_0':0, 				#Initial yaw coefficient
    				'Cn_beta':0.065, 			#Effect of slide angle of aircraft
    				'Cn_p':-0.03, 				#Effect of pitch rate
    				'Cn_r':-0.099, 				#Effect of yaw rate
    				'Cn_da':-0.053, 			#Effect of aileron
    				'Cn_dr':-0.0657, 			#Effect of rudder 
                    }
			
                    
                 
                    
