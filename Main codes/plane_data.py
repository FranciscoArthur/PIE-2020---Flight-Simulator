import numpy as np


def plane_data_fct(plane_position, plane_orientation,
               plane_speed, plane_angular_speed,atmospheric_parameters,plane_intrinsic_data,
               wind, pilot_data, plane_TAS_before_update, plane_TAS_vector_before_update):
    """
        Inputs: -plane_position: vector 3*1 [x, y, z]'
                -plane_orientation: vector 3*1 
                -plane_speed: vector 3*1 [vx, vy, vz]'
                -plane_angular_speed: vector 3*1 
                -plane_intrinsic_data: vectors babsed on aircraft file
                -wind: vector 3*1 [vx, vy, vz]'
                -pilot_data: vector 4*1 [Throttle, rudder, aileron, elevator]'
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
    v_air_vec = plane_TAS_vector_before_update

    # alpha
    alpha = np.arctan(v_air_vec[2] / abs(v_air_vec[0]))

    # beta
    beta = np.arcsin(v_air_vec[1] / v_air_mod)

    # Define coefficients of multiplication for contribution of rotations
    b2v = plane_intrinsic_data['span'] / (2 * v_air_mod)
    c_bar2v = plane_intrinsic_data['chord'] / (2 * v_air_mod)

    # Define control parameters angle
    q = plane_angular_speed[2]               # pitch angle
    p = plane_angular_speed[1]               # roll angle
    r = plane_angular_speed[0]               # yaw angle    
    
    de = pilot_data[3] / 20 * plane_intrinsic_data['de_max'] * d2r
    da = pilot_data[2] / 20 * plane_intrinsic_data['da_max'] * d2r
    dr = pilot_data[1] / 20 * plane_intrinsic_data['dr_max'] * d2r
    dthrust=pilot_data[0]/10


    # Lift coefficient
    cL = plane_intrinsic_data['CL_0'] + plane_intrinsic_data['CL_a'] * alpha + plane_intrinsic_data['CL_q'] * q * c_bar2v + plane_intrinsic_data['CL_de'] * de

    # Drag coefficient
    cd = plane_intrinsic_data['CD_0'] + plane_intrinsic_data['induced_drag_factor'] * cL ** 2 + plane_intrinsic_data['CD_de'] * de

    # Side force coefficient
    cy = plane_intrinsic_data['CY_0'] + plane_intrinsic_data['CY_beta'] * beta + (plane_intrinsic_data['CY_p'] * p + plane_intrinsic_data['CY_r'] * r) * b2v + plane_intrinsic_data['CY_dr'] * dr

    # Moment characteristics

    # Pitching moment
    # cm = plane_intrinsic_data['Cm_0'] + plane_intrinsic_data['Cm_da'] * da + plane_intrinsic_data['Cm_q'] * c_bar2v * q + plane_intrinsic_data['Cm_de'] * de
    cm = 0
    cl = 0
    cn = 0

    # Rolling moment
    # cl = plane_intrinsic_data['Cl_0'] + plane_intrinsic_data['Cl_da'] * da + plane_intrinsic_data['Cl_beta'] * beta + (
    #         plane_intrinsic_data['Cl_r'] * r + plane_intrinsic_data['Cl_p'] * p) * b2v * plane_intrinsic_data['Cl_dr'] * dr

    # Yawing moment
    # cn = plane_intrinsic_data['Cn_0'] + plane_intrinsic_data['Cn_beta'] * beta + (plane_intrinsic_data['Cn_p'] * p + plane_intrinsic_data['Cn_r'] * r) * b2v + \
    #      plane_intrinsic_data['Cn_da'] * da + plane_intrinsic_data['Cn_dr'] * dr


    #Thrust
    air_density=atmospheric_parameters[4]
    thrust=dthrust*plane_intrinsic_data['static_thrust']*(air_density/1.225)#*(1-np.exp((plane_position[2]-18000)/2000))    

    return [cL, cd, cy, cl, cm, cn, thrust]
