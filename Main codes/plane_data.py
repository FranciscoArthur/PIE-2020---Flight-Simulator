import numpy as np
import json
import importlib
import atmospheric_parameters


def plane_data(plane_position, plane_orientation,
               plane_speed, plane_angular_speed,
               wind, pilot_data,plane):
    """
        Inputs: -plane_position: vector 3*1 [x, y, z]'
                -plane_orientation: vector 3*1 
                -plane_speed: vector 3*1 [vx, vy, vz]'
                -plane_angular_speed: vector 3*1 
                -wind: vector 3*1 [vx, vy, vz]'
                -pilot_data: vector 4*1 [Throttle, elevator, aileron, rudder]'
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
    plane_file='Aircrafts.'+plane
    planemod=importlib.import_module(plane_file)

    # Speed of the airplane
    global_speed = plane_speed + wind
    v = np.sqrt(global_speed[0] ** 2 + global_speed[1] ** 2 + global_speed[2] ** 2)

    # alpha
    alpha = np.arctan(global_speed[2] / abs(global_speed[0]))

    # beta
    beta = np.arcsin(global_speed[1] / v)

    # Define coefficients of multiplication for contribution of rotations
    b2v = planemod.data['span'] / (2 * v)
    c_bar2v = planemod.data['chord'] / (2 * v)

    # Define control parameters angle
    q = plane_angular_speed[0]
    p = plane_angular_speed[1]
    r = plane_angular_speed[2]
    de = pilot_data[1] / 20 * planemod.data['de_max'] * d2r
    da = pilot_data[2] / 20 * planemod.data['da_max'] * d2r
    dr = pilot_data[3] / 20 * planemod.data['dr_max'] * d2r
    dthrust=pilot_data[0]/10


    # Lift coefficient
    cl = planemod.data['CL_0'] + planemod.data['CL_a'] * alpha + planemod.data['CL_q'] * q * c_bar2v + planemod.data[
        'CL_de'] * de

    # Drag coefficient
    cd = planemod.data['CD_0'] + planemod.data['induced_drag_factor'] * cl ** 2 + planemod.data['CD_de'] * de

    # Side force coefficient
    cy = planemod.data['CY_0'] + planemod.data['CY_beta'] * beta + (planemod.data['CY_p'] * p + planemod.data['CY_r'] * r) * b2v + \
         planemod.data['CY_dr'] * dr

    # Moment characteristics

    # Pitching moment
    cm = planemod.data['Cm_0'] + planemod.data['Cm_da'] * da + planemod.data['Cm_q'] * c_bar2v * q + planemod.data[
        'Cm_de'] * de

    # Rolling moment
    cl = planemod.data['Cl_0'] + planemod.data['Cl_da'] * da + planemod.data['Cl_beta'] * beta + (
            planemod.data['Cl_r'] * r + planemod.data['Cl_p'] * p) * b2v * planemod.data['Cl_dr'] * dr

    # Yawing moment
    cn = planemod.data['Cn_0'] + planemod.data['Cn_beta'] * beta + (planemod.data['Cn_p'] * p + planemod.data['Cn_r'] * r) * b2v + \
         planemod.data['Cn_da'] * da + planemod.data['Cn_dr'] * dr


    #Thrust
    atm_params=atmospheric_parameters.atmospheric_parameters_fct(plane_position[2])
    air_density=atm_params[4]
    thrust=dthrust*planemod.data['static_thrust']*(air_density/1.225)#*(1-np.exp((plane_position[2]-18000)/2000))    

    return [cl, cd, cy, cl, cm, cn, thrust]
