# -*- coding: utf-8 -*-
"""
@authors: PIE n°7 group - ISAE Supaero - 2020/2021

Description : This script intends to provide the current aerodynamic coefficients 
of the plane, which depends on the current state vector of the plane, the pilot
commands, and some intrinsic parameters of the plane. 

Source : JSBSim
"""

import numpy as np


def plane_data_fct(plane_position, plane_orientation,
               plane_speed, plane_angular_speed,atmospheric_parameters,plane_intrinsic_data,
               wind, pilot_data, plane_TAS_before_update, plane_TAS_vector_before_update,
               alpha, beta,plane_mach):
    """
        Inputs: -plane_position: vector 3*1 [x, y, z]'
                -plane_orientation: vector 3*1 
                -plane_speed: vector 3*1 [vx, vy, vz]'
                -plane_angular_speed: vector 3*1 
                -atmospheric_parameters
                -plane_intrinsic_data: vectors babsed on aircraft file
                -wind: vector 3*1 [vx, vy, vz]'
                -pilot_data: vector 4*1 [Throttle, rudder, aileron, elevator]'
                -true air speed : vector and module
                -aerodynamic angles : alpha and beta
                -Mach
        Outputs:
                -Lift coefficient: CL
                -Drag coefficient: CD
                -Side force coefficient: CY
                -Pitching moment: Cl
                -Rolling moment: Cm
                -Yawing moment: Cn
                -thrust
    """
    d2r = np.pi / 180
    r2d = 1 / d2r

    # Speed of the airplane in the air
    v_air_mod = plane_TAS_before_update


    # Define coefficients of multiplication for contribution of rotations
    b2v = plane_intrinsic_data['span'] / (2 * v_air_mod)
    c_bar2v = plane_intrinsic_data['chord'] / (2 * v_air_mod)

    # Define control parameters angle
    q = plane_angular_speed[2]               # pitch rate angular speed
    p = plane_angular_speed[1]               # roll rate angular speed
    r = plane_angular_speed[0]               # yaw rate angular speed     
    
    # Absolute pilot commands
    de = pilot_data[3] / 10 * plane_intrinsic_data['de_max'] * d2r
    da = pilot_data[2] / 10 * plane_intrinsic_data['da_max'] * d2r
    dr = pilot_data[1] / 10 * plane_intrinsic_data['dr_max'] * d2r
    dthrust=pilot_data[0]/10
    #Prandtl factor for mach effect
    prandtl=1/np.sqrt(1-plane_mach**2)
    
    ### Force coefficients
    # Lift coefficient
    cL = (plane_intrinsic_data['CL_0'] + plane_intrinsic_data['CL_a'] * alpha + plane_intrinsic_data['CL_q'] * q * c_bar2v + plane_intrinsic_data['CL_de'] * de)*prandtl

    # Drag coefficient
    cd = (plane_intrinsic_data['CD_0'] + plane_intrinsic_data['induced_drag_factor'] * cL ** 2 + plane_intrinsic_data['CD_de'] * de)*prandtl

    # Side force coefficient
    cy = (plane_intrinsic_data['CY_0'] + plane_intrinsic_data['CY_beta'] * beta + (plane_intrinsic_data['CY_p'] * p + plane_intrinsic_data['CY_r'] * r) * b2v + plane_intrinsic_data['CY_dr'] * dr)*prandtl




    # Moment coefficients
    # Pitching moment
    cm = (plane_intrinsic_data['Cm_0'] + plane_intrinsic_data['Cm_a'] * alpha + plane_intrinsic_data['Cm_q'] * c_bar2v * q + plane_intrinsic_data['Cm_de'] * de)*prandtl


    # Rolling moment
    cl = (plane_intrinsic_data['Cl_0'] + plane_intrinsic_data['Cl_da'] * da + plane_intrinsic_data['Cl_beta'] * beta + (
            plane_intrinsic_data['Cl_r'] * r + plane_intrinsic_data['Cl_p'] * p) * b2v * plane_intrinsic_data['Cl_dr'] * dr)*prandtl

    # Yawing moment
    cn = (plane_intrinsic_data['Cn_0'] + plane_intrinsic_data['Cn_beta'] * beta + (plane_intrinsic_data['Cn_p'] * p + plane_intrinsic_data['Cn_r'] * r) * b2v + 
          plane_intrinsic_data['Cn_da'] * da + plane_intrinsic_data['Cn_dr'] * dr)*prandtl



    ### Thrust
    air_density=atmospheric_parameters[4]
    thrust=dthrust*plane_intrinsic_data['static_thrust']*(air_density/1.225)*(1-np.exp((plane_position[2]-18000)/2000))    
    return [cL, cd, cy, cl, cm, cn, thrust]
