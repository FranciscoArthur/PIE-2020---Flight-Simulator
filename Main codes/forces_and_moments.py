import math;
import numpy as np;

#Forces calculation
def forces_calculation_fct(plane_mass, plane_TAS, plane_orientation, atmospheric_parameters_before_update, plane_data, plane_intrinsic_data):
    
    #collection of aerodynamic coefficients
    cz = plane_data[0];
    cx = plane_data[1];
    cy = plane_data[2];
    cl = plane_data[3];
    cm = plane_data[4];
    cn = plane_data[5];
    thrust = plane_data[6];
    
    #atmosphere data
    rho = atmospheric_parameters_before_update[4];
    g = atmospheric_parameters_before_update[6];
    
    #plane geometry              
    S = plane_intrinsic_data[Sw];   # S = wing surface
    m = plane_mass;
    
    #state vector
    v = plane_TAS;
    theta = plane_orientation[0];
    phi = plane_orientation[1];
        
    #Calculation
    drag = 0.5 * rho * S * cx * (v ** 2);

    lat_force = 0.5 * rho * S * cy * (v ** 2);

    weight = m * g;

    lift = 0.5 * rho * S * cz * (v ** 2);
    
    f_x = thrust - drag - weight * math.sin(theta);

    f_y = lat_force + weight * math.sin(phi) * np.sign(phi);

    f_z = lift - weight * math.cos(theta);
    
    return [f_x, f_y, f_z];


#Moments calculation
def moments_calculation_fct(plane_mass, plane_TAS, plane_orientation, atmospheric_parameters_before_update, plane_data, plane_intrinsic_data):
    
    #collection of aerodynamic coefficients
    cl = plane_data[3];
    cm = plane_data[4];
    cn = plane_data[5];
    
    #atmosphere data
    rho = atmospheric_parameters_before_update[4];
    
    #plane geometry
    with open('../Aircrafts/c172.json) as json_file:
              data = json.load(json_file)
              
    S = plane_intrinsic_data[Sw];   # S = wing surface
    b = plane_intrinsic_data[span];  # b = wingspan
    c = plane_intrinsic_data[chord];  # c = mean aerodynamic chord
    m = plane_mass;
    
    #state vector
    v = plane_TAS;
        
    #Calculation
    
    roll_m = 0.5 * rho * S * b * cl * (v ** 2);

    pitch_m = 0.5 * rho * S * c * cm * (v ** 2);

    yaw_m = 0.5 * rho * S * c * cn * (v ** 2);
    
    return [pitch_m, roll_m, yaw_m];


#Fuel consumption calculation
def fuel_consumption_calculation_fct(engine_data, current_pilot_controls, atmospheric_parameters_before_update):
    
    
    
    return plane_current_fuel_consumption;


#Load factor calculation
def loadfactor_calculation_fct(plane_mass, plane_current_forces, atmospheric_parameters_before_update, plane_orientation):
    
    #environment data
    m = plane_mass;
    g = atmospheric_parameters_before_update[6];
    
    #plane orientation
    theta = plane_orientation[0];
    phi = plane_orientation[1];
    psi = plane_orientation[2];
    
    #Forces
    f_x_a = plane_current_forces[0];
    f_y_a = plane_current_forces[1];
    f_z_a = plane_current_forces[2];
    weight = [0, 0, -m*g];
    
    #Conversion to terrestrial referential
    f_x_ext = f_x_a * cos(theta) * cos(psi) + f_y_a * sin(psi) + f_z_a * sin(theta);
    f_y_ext = f_x_a * sin(psi) + f_y_a * cos(psi) * cos(phi) + f_z_a * sin(phi);
    f_z_ext = f_x_a * sin(theta) + f_y_a * sin(phi) + f_z_a * cos(theta) * cos(phi);
    F_ext = [f_x_ext, f_y_ext, f_z_ext];
    
    #calculation
    n = [0, 0, 0];
    for i in range(3):
        n[i] = (weight[i] - F_ext[i]) / (-weight[3]);
    
    load_factor = sqrt(n[0]**2 + n[1]**2 + n[2]**2);
    
    return [n, load_factor];
    
    