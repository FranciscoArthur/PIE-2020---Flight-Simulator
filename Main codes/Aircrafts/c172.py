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
                    'mass': 1150,                                             # Aircraft mass [kg] 
                    'inertia' : np.diag([948, 1346, 1967]) * slugft2_2_kgm2,  # Aircraft inertia tensor [kg·m²]
 
                    # Thrust
                    'static_thrust':1000,                                     # [N] 
                    
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
				    'CL_0':0.31,
    				'CL_a':5.143,                                             
    				'CL_q':3.9,
    				'CL_de':0.43,
    			    # Drag
    				'CD_0':0.031,
    				'induced_drag_factor':0.054,
    				'CD_de':0.43,
    			    # Side_force
    				'CY_0':0,
    				'CY_beta':-0.31,
    				'CY_p':-0.037,
    				'CY_r':0.21,
    				'CY_dr':0.187,
    			    # Pitch
    				'Cm_0':-0.015,
    				'Cm_da':-0.89,
    				'Cm_q':-12.4,
    				'Cm_de':-1.28,
    			    # Roll
    				'Cl_0':0,
    				'Cl_beta':-0.089,
    				'Cl_p':-0.47,
    				'Cl_r':0.096,
    				'Cl_da':-0.178,
    				'Cl_dr':0.0147,
    			    # Yaw
    				'Cn_0':0,
    				'Cn_beta':0.065,
    				'Cn_p':-0.03,
    				'Cn_r':-0.099,
    				'Cn_da':-0.053,
    				'Cn_dr':-0.0657,
                    }
			
                    
                 
                    