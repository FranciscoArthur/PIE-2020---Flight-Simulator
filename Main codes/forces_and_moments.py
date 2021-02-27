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
    psi = plane_orientation[0];   # yaw angle [rad]
    phi = plane_orientation[1];   # roll angle [rad]
    theta = plane_orientation[2]; # pitch angle [rad]
        
    #Calculation
    drag = 0.5 * rho * S * cd * (v ** 2);

    lat_force = 0.5 * rho * S * cy * (v ** 2);

    weight = m * g;

    lift = 0.5 * rho * S * cz * (v ** 2);
    
    
    ### Resulting force - in the airplane frame
    # Test without weight - add it in the flight_equations
    # weight is already considered in "flight_equations"
    # f_x_plane = thrust - drag # - weight * math.sin(theta); 
    # f_y_plane = lat_force # + weight * math.sin(phi) * np.sign(phi);
    # f_z_plane = lift # - weight * math.cos(theta);
    
    # Test with weight - Arthur style
    f_x_plane = thrust - drag - weight * math.sin(theta);
    f_y_plane = lat_force + weight * math.sin(phi) * math.cos(theta);
    f_z_plane = lift - weight * math.cos(theta) * math.cos(phi);
    
    # Test with weight - Thomas style
    # f_x_plane = thrust - drag - weight * math.sin(theta);
    # f_y_plane = lat_force + weight * math.sin(phi) * np.sign(phi);
    # f_z_plane = lift - weight * math.cos(theta);    
    
    return [f_x_plane, f_y_plane, f_z_plane];


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
    psi = plane_orientation[0];   # yaw angle [rad]
    phi = plane_orientation[1];   # roll angle [rad]
    theta = plane_orientation[2]; # pitch angle [rad]

    # Force - conversion to terrestrial referential
    from coordinates_transformation import body2hor_fct
    F_ext = body2hor_fct(plane_current_forces, theta, phi, psi)    # Change of frame - from body frame to flat heart frame
    
    weight = [0, 0, -m*g];
    
    #calculation
    n = [0, 0, 0];
    for i in range(3):
        n[i] = (F_ext[i] - weight[i]) / (weight[2]);
    
    load_factor = math.sqrt(n[0]**2 + n[1]**2 + n[2]**2);
    
    return [n, load_factor];
    
    
