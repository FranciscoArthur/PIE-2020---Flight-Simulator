# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 17:37:32 2021

----------
Boeing 747
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
                    'mass': 183500,                                           # Aircraft mass [kg] 
                    'inertia' : (np.diag([18200000, 33100000, 49700000]) + 
                                 [[0, 0, 970000],[0, 0, 0], [970000, 0, 0]])
                                 * slugft2_2_kgm2,                            # Aircraft inertia tensor [kg·m²]
 
                    # Thrust
                    'static_thrust':240000,                                   # [N] 
                    
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
				    'CL_0':0.21,
    				'CL_a':4.4,                                             
    				'CL_q':6.6,
    				'CL_de':0.32,
    			    # Drag
    				'CD_0':0.0164,
    				'induced_drag_factor':0.054,
    				'CD_de':0.43,
    			    # Side_force
    				'CY_0':0,
    				'CY_beta':-0.9,
    				'CY_p':0,
    				'CY_r':0,
    				'CY_dr':0.12,
    			    # Pitch
    				'Cm_0':0,
    				'Cm_da':-1,
    				'Cm_q':-20.5,
    				'Cm_de':-1.3,
    			    # Roll
    				'Cl_0':0,
    				'Cl_beta':-0.16,
    				'Cl_p':-0.34,
    				'Cl_r':0.13,
    				'Cl_da':-0.013,
    				'Cl_dr':0.008,
    			    # Yaw
    				'Cn_0':0,
    				'Cn_beta':0.16,
    				'Cn_p':-0.026,
    				'Cn_r':-0.280,
    				'Cn_da':-0.0018,
    				'Cn_dr':-0.1,
                    }
			