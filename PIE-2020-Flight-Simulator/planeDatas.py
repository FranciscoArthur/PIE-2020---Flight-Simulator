import numpy as np
import json

def planeDatas(plane_position, plane_orientation,
               plane_speed, plane_angular_speed,
               wind, pilot_data):
    """
        Inputs: -plane_position: vector 3*1 [x, y, z]
                -plane_orientation: vector 3*1 
                -plane_speed: vector 3*1 [vx, vy, vz]
                -plane_angular_speed: vector 3*1 
                -wind: vector 3*1 [vx, vy, vz]
                -pilot_data: vector 4*1 [Throttle, elevator, aileron, rudder]
        Outputs:
                -Lift coefficient: CL
                -Drag coefficient: CD
                -Side force coefficient: CY
                -Pitching moment: Cl
                -Rolling moment: Cm
                -Yawing moment: Cn
    """
    D2R=np.pi/180
    R2D=1/D2R
    plane='cessna_172'

    #load datas from the plane
    with open('planes.json') as json_file:
        data=json.load(json_file)
    
    
    #Speed of plane
    global_speed=plane_speed+wind
    V       =   np.sqrt(global_speed[0]**2+global_speed[1]**2+global_speed[2]**2)
    #alpha
    alpha=np.arctan(global_speed[2]/abs(global_speed[0]))
    #beta
    beta=np.arcsin(global_speed[1]/V)
    
    #Define coefficients of multiplication for contribution of rotations
    b_2V=data[plane]['geometry']['bw']/(2*V)
    cBar_2V=data[plane]['geometry']['cBar']/(2*V)

    #Define control parameters angle
    q=plane_angular_speed[0]
    p=plane_angular_speed[1]
    r=plane_angular_speed[2]
    de=pilot_data[1]/20*data[plane]['amplitudes']['de']*D2R
    da=pilot_data[2]/20*data[plane]['amplitudes']['da']*D2R
    dr=pilot_data[3]/20*data[plane]['amplitudes']['dr']*D2R
    
    # Force characteristics
    coeffs=data[plane]['force_coefficients']
        # Lift coefficient
    coeffs_lift=coeffs['lift']
    CL=coeffs_lift['CL_0']+coeffs_lift['CL_a']*alpha+coeffs_lift['CL_q']*q*cBar_2V+coeffs_lift['CL_de']*de

        #Drag coefficient
    coeffs_drag=coeffs['drag']
    CD=coeffs_drag['CD_0']+coeffs_drag['induced_drag_factor']*CL**2+coeffs_drag['CD_de']*de

        #Side force coefficient
    coeffs_sf=coeffs['side_force']
    CY = coeffs_sf['CY_0']+coeffs_sf['CY_beta']*beta+(coeffs_sf['CY_p']*p+coeffs_sf['CY_r']*r)*b_2V+coeffs_sf['CY_dr']*dr

    #Moment characteristics
        # Pitching moment
    coeffs_pitch=coeffs['pitch']
    Cm=coeffs_pitch['Cm_0']+coeffs_pitch['Cm_da']*da+coeffs_pitch['Cm_q']*cBar_2V*q+coeffs_pitch['Cm_de']*de
    

        #Rolling moment
    coeffs_roll=coeffs['roll']
    Cl=coeffs_roll['Cl_0']+coeffs_roll['Cl_da']*da+coeffs_roll['Cl_beta']*beta+(coeffs_roll['Cl_r']*r+coeffs_roll['Cl_p']*p)*b_2V*coeffs_roll['Cl_dr']*dr

        #Yawing moment
    coeffs_yaw=coeffs['yaw']
    Cn=coeffs_yaw['Cn_0']+coeffs_yaw['Cn_beta']*beta+(coeffs_yaw['Cn_p']*p+coeffs_yaw['Cn_r']*r)*b_2V+coeffs_yaw['Cn_da']*da+coeffs_yaw['Cn_dr']*dr
    
    return([CL,CD,CY,Cl,Cm,Cn])
