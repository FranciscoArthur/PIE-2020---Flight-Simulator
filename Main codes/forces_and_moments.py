# -*- coding: utf-8 -*-
"""
@authors: PIE n°7 group - ISAE Supaero - 2020/2021

Description : This script intends to provide the necessary functions to calculate 
the current resulting force and moment, the load factor and the fuel consumption.  

"""


import math; 
import numpy as np;


#Forces calculation
def forces_calculation_fct(plane_mass, plane_TAS,plane_TAS_vector, plane_orientation, atmospheric_parameters_before_update, plane_data, plane_intrinsic_data,hor2bodymatrix):
    
    # Collection of aerodynamic coefficients
    CL = plane_data[0];
    CD = plane_data[1];
    CY = plane_data[2];
    thrust = plane_data[6];
    
    # Atmosphere data
    rho = atmospheric_parameters_before_update[4];
    g = np.array([0,0,atmospheric_parameters_before_update[6]]);
    
    # Gravity in body frame
    gb=hor2bodymatrix.dot(g)
    # Plane geometry              
    S = plane_intrinsic_data['Sw'];   # S = wing surface
    m = plane_mass;

    # Dynamic pressure and surface
    qbarS = 0.5*rho*plane_TAS**2*S
    
    # State vector
    psi = plane_orientation[0];   # yaw angle [rad]
    phi = plane_orientation[1];   # roll angle [rad]
    theta = plane_orientation[2]; # pitch angle [rad]

    # Angle of attack
    alpha=np.arctan(plane_TAS_vector[2]/plane_TAS_vector[0])

    # Coefficients on body X and Z coefficient
    CX=-CD*np.cos(alpha)+CL*np.sin(alpha)
    CZ=-CD*np.sin(alpha)-CL*np.cos(alpha)

    # State accelerations in plane referential
    f_x_plane = (CX*qbarS+thrust)+m*gb[0]
    f_y_plane = CY*qbarS+m*gb[1]
    f_z_plane = CZ*qbarS +m*gb[2]
    return [f_x_plane, f_y_plane, f_z_plane];


#Moments calculation
def moments_calculation_fct(plane_mass, plane_TAS, plane_orientation, atmospheric_parameters_before_update, plane_data, plane_intrinsic_data):
    
    # Collection of aerodynamic coefficients
    cl = plane_data[3];
    cm = plane_data[4];
    cn = plane_data[5];
    
    # Atmosphere data
    rho = atmospheric_parameters_before_update[4];
    
    # Plane geometry
    S = plane_intrinsic_data['Sw'];   # S = wing surface
    b = plane_intrinsic_data['span'];  # b = wingspan
    c = plane_intrinsic_data['chord'];  # c = mean aerodynamic chord
    m = plane_mass;
    
    # State vector
    v = plane_TAS;
        
    # Calculation
    roll_m =  0.5 * rho * S * b * cl * (v ** 2);
    pitch_m = 0.5 * rho * S * c * cm * (v ** 2);
    yaw_m = 0.5 * rho * S * b * cn * (v ** 2);
    
    return [yaw_m, roll_m, pitch_m];


#Fuel consumption calculation
def fuel_consumption_calculation_fct(current_throttle, plane_intrinsic_data):
    min_consum = plane_intrinsic_data['min_consumption_zero_throttle']
    max_consum = plane_intrinsic_data['max_consumption_full_throttle']
    
    ff = (min_consum + max_consum*current_throttle/10)/3600         # [kg/s]
    
    return ff


#Load factor calculation
def loadfactor_calculation_fct(plane_mass, plane_current_forces, atmospheric_parameters_before_update, plane_orientation):
    
    # Environment data
    m = plane_mass;
    g = atmospheric_parameters_before_update[6];
    
    # Plane orientation
    psi = plane_orientation[0];   # yaw angle [rad]
    phi = plane_orientation[1];   # roll angle [rad]
    theta = plane_orientation[2]; # pitch angle [rad]

    # Force - conversion to terrestrial referential
    from coordinates_transformation import body2hor_fct
    F_ext = body2hor_fct(plane_current_forces, theta, phi, psi)    # Change of frame - from body frame to flat heart frame
    
    weight = [0, 0, -m*g];
    
    # Calculation
    n = [0, 0, 0];
    for i in range(3):
        n[i] = (F_ext[i] - weight[i]) / (weight[2]);
    
    load_factor = math.sqrt(n[0]**2 + n[1]**2 + n[2]**2);
    
    return [n, load_factor];









