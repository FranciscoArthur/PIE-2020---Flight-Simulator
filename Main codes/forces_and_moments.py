import math; 
import numpy as np;

#Forces calculation
def forces_calculation_fct(plane_mass, plane_TAS, plane_orientation, atmospheric_parameters_before_update, plane_data, plane_intrinsic_data):
    
    #collection of aerodynamic coefficients
    cz = plane_data[0];
    cd = plane_data[1];
    cy = plane_data[2];
    cl = plane_data[3];
    cm = plane_data[4];
    cn = plane_data[5];
    thrust = plane_data[6];
    
    #atmosphere data
    rho = atmospheric_parameters_before_update[4];
    g = atmospheric_parameters_before_update[6];
    
    #plane geometry              
    S = plane_intrinsic_data['Sw'];   # S = wing surface
    m = plane_mass;
    
    #state vector
    v = plane_TAS;
    psi = plane_orientation[0];   # yaw angle
    phi = plane_orientation[1];   # roll angle
    theta = plane_orientation[2]; # pitch angle
        
    #Calculation
    drag = 0.5 * rho * S * cd * (v ** 2);

    lat_force = 0.5 * rho * S * cy * (v ** 2);

    weight = m * g;

    lift = 0.5 * rho * S * cz * (v ** 2);
    
    
    ### Resulting force - in the airplane frame
    # Test without weight - add it in the flight_equations
    # weight is already considered in "flight_equations"
    # f_x = thrust - drag # - weight * math.sin(theta); 
    # f_y = lat_force # + weight * math.sin(phi) * np.sign(phi);
    # f_z = lift # - weight * math.cos(theta);
    
    # Test with weight - Arthur style
    # f_x = thrust - drag - weight * math.sin(theta);
    # f_y = lat_force + weight * math.sin(phi) * math.cos(theta);
    # f_z = lift - weight * math.cos(theta) * math.cos(phi);
    
    # Test with weight - Thomas style
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
              
    S = plane_intrinsic_data['Sw'];   # S = wing surface
    b = plane_intrinsic_data['span'];  # b = wingspan
    c = plane_intrinsic_data['chord'];  # c = mean aerodynamic chord
    m = plane_mass;
    
    #state vector
    v = plane_TAS;
        
    #Calculation
    
    roll_m =  0.5 * rho * S * b * cl * (v ** 2);

    pitch_m = 0.5 * rho * S * c * cm * (v ** 2);

    yaw_m = 0.5 * rho * S * c * cn * (v ** 2);
    
    return [yaw_m, roll_m, pitch_m];


#Fuel consumption calculation
def fuel_consumption_calculation_fct(current_throttle, plane_intrinsic_data):
    min_consum = plane_intrinsic_data['min_consumption_zero_throttle']
    max_consum = plane_intrinsic_data['max_consumption_full_throttle']
    
    ff = (min_consum + max_consum*current_throttle/10)/3600         # [kg/s]
    
    return ff


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
    f_x_ext = f_x_a * math.cos(theta) * math.cos(psi) + f_y_a * math.sin(psi) + f_z_a * math.sin(theta);
    f_y_ext = f_x_a * math.sin(psi) + f_y_a * math.cos(psi) * math.cos(phi) + f_z_a * math.sin(phi);
    f_z_ext = f_x_a * math.sin(theta) + f_y_a * math.sin(phi) + f_z_a * math.cos(theta) * math.cos(phi);
    F_ext = [f_x_ext, f_y_ext, f_z_ext];
    
    #calculation
    n = [0, 0, 0];
    for i in range(3):
        n[i] = (weight[i] - F_ext[i]) / (-weight[2]);
    
    load_factor = math.sqrt(n[0]**2 + n[1]**2 + n[2]**2);
    
    return [n, load_factor];
    
    
