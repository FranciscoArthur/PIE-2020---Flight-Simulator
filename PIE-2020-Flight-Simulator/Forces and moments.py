import math;
import numpy as np;


##Thrust
def thrust(rho, D, n, C_t):                               #D = diameter and n = rotation speed
    return rho * C_t * (n**2) * (D**4);

##Function
def FandM(coeffs_aero, plane_data, plane_speed, atmosphere_data, plane_orientation):
    
    g = 9.81;
    
    rho = atmosphere_data[4];
    
    S = plane_data[0];
    b = plane_data[1];             #b = wingspan
    c = plane_data[2];             #c = mean aerodynamic chord
    l = plane_data[3];             #l = mean aerodynamic chord of VTP
    m = plane_data[4];
    
    v = math.sqrt( plane_speed[0]**2 + plane_speed[1]**2 + plane_speed[2]**2);
    
    Cz = coeffs_aero[0];
    Cx = coeffs_aero[1];
    Cy = coeffs_aero[2];
    Cl = coeffs_aero[3];
    Cm = coeffs_aero[4];
    Cn = coeffs_aero[5];
    
    theta = plane_orientation[0];
    psi = plane_orientation[1];
    phi = plane_orientation[2];
    
    drag = 0.5 * rho * S * Cx * (v**2);
    
    lat_force = 0.5 * rho * S * Cy * (v**2);
    
    weight = m * g;
    
    lift = 0.5 * rho * S * Cz * (v**2);
        
    roll_m = 0.5 * rho * S * b * Cl * (v**2);
    
    pitch_m = 0.5 * rho * S * c * Cm * (v**2);
    
    yaw_m = 0.5 * rho * S * l * Cn * (v**2);
    
    thrust = 0; ##trouver modèle de calcul correct pour la traction
    
    Fx = thrust - drag - weight * math.sin(theta);
    
    Fy = lat_force + weight * math.sin(phi) * np.sign(phi);
    
    Fz = lift - weight * math.cos(theta);
    
    return [Fx, Fy, Fz, pitch_m, roll_m, yaw_m];
    