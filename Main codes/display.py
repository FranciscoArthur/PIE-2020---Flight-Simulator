# -*- coding: utf-8 -*-
"""
@authors: PIE n°7 group - ISAE Supaero - 2020/2021

Description : This script intends to provide the necessary function to display 
the results through the time evolution of the different parameters, and to save
the results in an excel file

"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
import xlsxwriter


def display_fct(result):
    """Plotting function that generates graphs for the results variables history.


    Input:
        Dictionnary of the form:
            result = {}
            result['plane_position'] = plane_position
            result['plane_orientation'] = plane_orientation
            result['plane_speed'] = plane_speed                    # Ground speed
            result['plane_TAS_vector'] = plane_TAS_vector          # Plane speed in the relative air - vector
            result['plane_TAS'] = plane_TAS                        # True AirSpeed - module
            result['plane_angular_speed'] =   plane_angular_speed
            result['plane_fuel_load'] = plane_fuel_load
            result['plane_mass'] = plane_mass
            result['plane_resulting_force'] = plane_resulting_force
            result['plane_resulting_moment'] = plane_resulting_moment
            result['plane_Mach'] = plane_Mach
            result['plane_qinf'] = plane_qinf
            result['time'] = time   
            result['pilot_commands'] = pilot_inputs
            result['plane_load_factor'] = plane_load_factor
            result['plane_load_factor_vector'] = plane_load_factor_vector 
            result['plane_intrinsic_data'] = plane_intrinsic_data
            result['plane_atmospheric_parameters'] = plane_atmospheric_parameters
            result['plane_aerodynamic_coefficients'] = plane_aerodynamic_coefficients
            result['plane_gamma_angle']=plane_gamma_angle
            result['plane_sigma_angle']=plane_sigma_angle
            result['plane_alpha_angle']=plane_alpha_angle
            result['plane_beta_angle']=plane_beta_angle
        """

    #-----------------------------------------------------------------------------
    # Selection of the desired plots
    # If the '1' value is given the graph is displayed
    
    GRAPH_Velocities =       1
    GRAPH_Positions =        1
    GRAPH_rotation_rates =   0
    GRAPH_angles =           0
    GRAPH_controls =         0
    GRAPH_TASandMach =       0
    GRAPH_Mass =             0
    GRAPH_LoadFactor =       0
    GRAPH_Forces =           0
    GRAPH_Moments =          0
    GRAPH_Coeffs =           0
    GRAPH_AeroAngles =       0
    
    #-----------------------------------------------------------------------------
    

        
    r2d = 180 / np.pi

    # Time coordinates
    t = result['time']                              


    # Definition of the state vector
    x = np.zeros((len(t),29))

    x[:,0] = result['plane_speed'][:,0]             # Body-axis x inertial velocity, ub, m/s 
    x[:,1] = result['plane_speed'][:,1]             # Body-axis y inertial velocity, vb, m/s
    x[:,2] = result['plane_speed'][:,2]             # Body-axis z inertial velocity, wb, m/s
    x[:,3] = result['plane_position'][:,0]          # North position of center of mass WRT Earth, xe, m
    x[:,4] = result['plane_position'][:,1]          # East position of center of mass WRT Earth, ye, m
    x[:,5] = -result['plane_position'][:,2]          # Negative of c.m. altitude WRT Earth, ze = -h, m
    x[:,6] = result['plane_angular_speed'][:,0]     # Body-axis yaw rate, rr, rad/s
    x[:,7] = result['plane_angular_speed'][:,1]     # Body-axis roll rate, pr, rad/s
    x[:,8] = result['plane_angular_speed'][:,2]     # Body-axis pitch rate, qr,rad/s
    x[:,9] = result['plane_orientation'][:,0]       # Yaw angle of body WRT Earth, rad
    x[:,10] = result['plane_orientation'][:,1]      # Roll angle of body WRT Earth, rad
    x[:,11] = result['plane_orientation'][:,2]      # Pitch angle of body WRT Earth, rad
    x[:,12] = result['plane_resulting_force'][:,0]
    x[:,13] = result['plane_resulting_force'][:,1]
    x[:,14] = result['plane_resulting_force'][:,2]
    x[:,15] = result['plane_resulting_moment'][:,0]
    x[:,16] = result['plane_resulting_moment'][:,1]
    x[:,17] = result['plane_resulting_moment'][:,2]
    x[:,18] = result['plane_aerodynamic_coefficients'][:,0] #cL
    x[:,19] = result['plane_aerodynamic_coefficients'][:,1] #cD
    x[:,20] = result['plane_aerodynamic_coefficients'][:,2] #cy
    x[:,21] = result['plane_aerodynamic_coefficients'][:,3] #cl
    x[:,22] = result['plane_aerodynamic_coefficients'][:,4] #cm
    x[:,23] = result['plane_aerodynamic_coefficients'][:,5] #cn
    x[:,24] = result['plane_aerodynamic_coefficients'][:,6] #Thrust
    x[:,25] = result['plane_alpha_angle']
    x[:,26] = result['plane_beta_angle']
    x[:,27] = result['plane_sigma_angle']
    x[:,28] = result['plane_gamma_angle']

    

    ### Plots : state vector
    if GRAPH_Velocities == 1:
        f1 = plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(t, x[:,0])
        plt.xlabel('Time, s'), plt.ylabel('Axial Velocity (u), m/s'), plt.grid(True)
        plt.title('Forward Body-Axis Component of Inertial Velocity, u')
        plt.subplot(2, 2, 2)
        plt.plot(t, x[:,1])
        plt.xlabel('Time, s'), plt.ylabel('Side Velocity (v), m/s'), plt.grid(True)
        plt.title('Side Body-Axis Component of Inertial Velocity, v')
        plt.subplot(2, 2, 3)
        plt.plot(t, x[:,2])
        plt.xlabel('Time, s'), plt.ylabel('Normal Velocity (w), m/s'), plt.grid(True)
        plt.title('Normal Body-Axis Component of Inertial Velocity, z')
        plt.subplot(2, 2, 4)
        plt.plot(t, x[:,0], t, x[:,1], t, x[:,2])
        plt.xlabel('Time, s'), plt.ylabel('u (blue), v (green), w (red), m/s'), plt.grid(True)
        plt.title('Body-Axis Component of Inertial Velocity')
        plt.legend(['Axial velocity, u', 'Side velocity, v', 'Normal velocity, w'])
        
        
    if GRAPH_Positions == 1:
        f2 = plt.figure()
        plt.subplot(3, 2, 1)
        plt.plot(t, x[:,3])
        plt.xlabel('Time, s'), plt.ylabel('North (x), m'), plt.grid(True)
        plt.title('North Location, x')
        
        plt.subplot(3, 2, 2)
        plt.plot(t, x[:,4])
        plt.xlabel('Time, s'), plt.ylabel('East (y), m'), plt.grid(True)
        plt.title('East Location, y')
        
        plt.subplot(3, 2, 3)
        plt.plot(t, x[:,5])
        plt.xlabel('Time, s'), plt.ylabel('Altitude (z), m'), plt.grid(True)
        plt.title('Altitude, z')
        
        plt.subplot(3, 2, 4)
        plt.plot((np.sqrt(x[:,3] * x[:,3] + x[:,4] * x[:,4])), x[:,5])
        plt.xlabel('Ground Range, m'), plt.ylabel('Altitude, m'), plt.grid(True)
        plt.title('Altitude vs. Ground Range')

        plt.subplot(3, 2, 5)
        plt.plot(x[:,3], x[:,4])
        plt.xlabel('North, m'), plt.ylabel('East, m'), plt.grid(True)
        plt.title('Ground Track, North vs. East')

        ax = f2.add_subplot(3, 2, 6, projection="3d")
        ax.plot3D(x[:,3], x[:,4], x[:,5])
        ax.set_xlabel('North, m'), ax.set_ylabel('East, m'), ax.set_zlabel('Altitude, m'), plt.grid(True)
        ax.set_title('3D Flight Path')



    if GRAPH_rotation_rates ==1:
        f3 = plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(t, x[:,6] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('Yaw Rate (p), deg/s'), plt.grid(True)
        plt.title('Body-Axis Yaw Component of Inertial Rate, p')
        plt.subplot(2, 2, 2)
        plt.plot(t, x[:,7] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('Roll Rate (q), deg/s'), plt.grid(True)
        plt.title('Body-Axis Roll Component of Inertial Rate, q')
        plt.subplot(2, 2, 3)
        plt.plot(t, x[:,8] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('Pitch Rate (r), deg/s'), plt.grid(True)
        plt.title('Body-Axis Pitch Component of Inertial Rate, r')
        plt.subplot(2, 2, 4)
        plt.plot(t, x[:,6] * r2d, t, x[:,7] * r2d, t, x[:,8] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('p (blue), q (green), r (red), deg/s'), plt.grid(True)
        plt.title('Body-Axis Inertial Rate Vector Components')
        plt.legend(['Yaw rate, r', 'Roll rate, p', 'Pitch rate, q'])




    if GRAPH_angles ==1:
        f4 = plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(t, x[:,9] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('Yaw Angle (psi), deg'), plt.grid(True)
        plt.title('Earth-Relative Yaw Attitude')
        plt.subplot(2, 2, 2)
        plt.plot(t, x[:,10] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('Roll Angle (phi), deg'), plt.grid(True)
        plt.title('Earth-Relative Roll Attitude')
        
        plt.subplot(2, 2, 3)
        plt.plot(t, x[:,11] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('Pitch Angle (theta, deg'), plt.grid(True)
        plt.title('Earth-Relative Pitch Attitude')
        plt.subplot(2, 2, 4)
        plt.plot(t, x[:,9] * r2d, t, x[:,10] * r2d, t, x[:,11] * r2d)
        plt.xlabel('Time, s'), plt.ylabel('psi (blue), phi (green), theta (red), deg'), plt.grid(True)
        plt.title('Euler Angles')
        plt.legend(['Yaw angle, psi', 'Roll angle, phi', 'Pitch angle, theta'])



    ### PLOT CONTROL HISTORY

    # Definition of the control vector

    u = np.zeros((7, len(t)))

    u[0] = result['pilot_commands'][0]  # Throttle, dT, %# Elevator, dEr, rad, positive: trailing edge down
    u[1] = result['pilot_commands'][1]  # Rudder, dRr, rad, positive: trailing edge left
    u[2] = result['pilot_commands'][2]  # Aileron, dAr, rad, positive: left trailing edge down
    u[3] = result['pilot_commands'][3]  # Elevator, dEr, rad, positive: trailing edge down
    u[4] = result['pilot_commands'][4]  # Air Breaks
    u[5] = result['pilot_commands'][5]  # High Lift
    u[6] = result['pilot_commands'][6]  # Landing Gear  
    
    
    if GRAPH_controls ==1:
    
        f5 =plt.figure()
    
        plt.subplot(1, 2, 1)
        plt.plot(t, u[0])
        plt.xlabel('Time, s'), plt.ylabel('Throttle Setting'), plt.grid(True)
        plt.title('Throttle Test Inputs')
        
        plt.subplot(1, 2, 2)
        plt.plot(t, u[2], t, u[1], t, u[3])
        plt.xlabel('Time, s'), plt.ylabel('Aileron (blue), Rudder (green), Elevator (red), deg'), plt.grid(True)
        plt.title('Control Surfaces Test Inputs'), plt.legend(['Aileron, dA', 'Rudder, dR', 'Elevator, dE'])
    



    # PLOT SPEED HISTORY

    # Definition of speed variables

    tas = result['plane_TAS']  # Speed module wrt the air, consideering the wind

    mach = result['plane_Mach']  # Mach number
    
    
    if GRAPH_TASandMach ==1:
        
        f6 = plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(t, tas)
        plt.xlabel('Time, s'), plt.ylabel('TAS'), plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(t, mach)
        plt.xlabel('Time, s'), plt.ylabel('Mach'), plt.grid(True)



    # PLOT MASS HISTORY

    mass = result['plane_mass']  # Plane mass


    if GRAPH_Mass ==1:
        f7 = plt.figure()
        plt.plot(t, mass)
        plt.xlabel('Time, s'), plt.ylabel('Plane Mass'), plt.grid(True)



    # PLOT LOAD FACTOR + FORCES + MOMENTS HISTORY

    nz = result['plane_load_factor']  # Plane load factor


    if GRAPH_LoadFactor ==1:
        f8 = plt.figure()
        plt.plot(t, nz)
        plt.xlabel('Time, s'), plt.ylabel('Load Factor'), plt.grid(True)
        
        
    if GRAPH_Forces == 1:
        
        f9 = plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(t, x[:,12])
        plt.xlabel('Time, s'), plt.ylabel('Resulting force in x [N]'), plt.grid(True)
        plt.subplot(2, 2, 2)
        plt.plot(t, x[:,13])
        plt.xlabel('Time, s'), plt.ylabel('Resulting force in y [N]'), plt.grid(True)
        plt.subplot(2, 2, 3)
        plt.plot(t, x[:,14])
        plt.xlabel('Time, s'), plt.ylabel('Resulting force in z [N]'), plt.grid(True)
        plt.subplot(2, 2, 4)
        plt.plot(t, x[:,24])
        plt.xlabel('Time, s'), plt.ylabel('Thrust [N]'), plt.grid(True)
        
    
    if GRAPH_Moments == 1:
        
        f10 = plt.figure()
        plt.subplot(1, 3, 1)
        plt.plot(t, x[:,15])
        plt.xlabel('Time, s'), plt.ylabel('Yaw moment [N.m]'), plt.grid(True)
        plt.subplot(1, 3, 2)
        plt.plot(t, x[:,16])
        plt.xlabel('Time, s'), plt.ylabel('Roll moment [N.m]'), plt.grid(True)
        plt.subplot(1, 3, 3)
        plt.plot(t, x[:,17])
        plt.xlabel('Time, s'), plt.ylabel('Pitch moment [N.m]'), plt.grid(True)
        
    
    if GRAPH_Coeffs == 1:
        
        f11 = plt.figure()
        plt.subplot(3, 3, 1)
        plt.plot(t, x[:,18])
        plt.xlabel('Time, s'), plt.ylabel('cL'), plt.grid(True)
        plt.subplot(3, 3, 2)
        plt.plot(t, x[:,19])
        plt.xlabel('Time, s'), plt.ylabel('cD'), plt.grid(True)
        plt.subplot(3, 3, 3)
        plt.plot(t, x[:,20])
        plt.xlabel('Time, s'), plt.ylabel('cy'), plt.grid(True)
        plt.subplot(3, 3, 4)
        plt.plot(t, x[:,21])
        plt.xlabel('Time, s'), plt.ylabel('cl'), plt.grid(True)
        plt.subplot(3, 3, 5)
        plt.plot(t, x[:,22])
        plt.xlabel('Time, s'), plt.ylabel('cm'), plt.grid(True)
        plt.subplot(3, 3, 6)
        plt.plot(t, x[:,23])
        plt.xlabel('Time, s'), plt.ylabel('cn'), plt.grid(True)
        
        
    if GRAPH_AeroAngles == 1:
        
        f10 = plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(t, x[:,25]*r2d)
        plt.xlabel('Time, s'), plt.ylabel('Alpha'), plt.grid(True)
        plt.subplot(2, 2, 2)
        plt.plot(t, x[:,26]*r2d)
        plt.xlabel('Time, s'), plt.ylabel('Beta'), plt.grid(True)
        plt.subplot(2, 2, 3)
        plt.plot(t, x[:,27]*r2d)
        plt.xlabel('Time, s'), plt.ylabel('Sigma'), plt.grid(True)
        plt.subplot(2, 2, 4)
        plt.plot(t, x[:,28]*r2d)
        plt.xlabel('Time, s'), plt.ylabel('Gamma'), plt.grid(True)
        
        
        
        
        
    
    ### Exporting results to a text file
    
    # Create file and worksheet    
    outWorkbook = xlsxwriter.Workbook("Results.xlsx")
    outSheet = outWorkbook.add_worksheet()
    
    # Write data results to file
    header = ['Time','Vx','Vy','Vy','North_pos','East_pos','Altitude','Yaw_rate','Roll_rate','Pitch_rate','Yaw_angle','Roll_angle','Pitch_angle','F_x','F_y','F_z','Yaw_moment','Roll_moment','Pitch_moment','CL','CD','Cy','Cl','Cm','Cn','Thrust','Alpha','Beta','Sigma','Gamma']
    
    for i in range(len(header)):
        outSheet.write(0,i, header[i])
        
    for i in range(len(t)):
        outSheet.write(i+1,0, t[i])
             
    for i in range(len(t)):
        for j in range(len(x[0,:])):
            outSheet.write(i+1,j+1,x[i,j])
                             
            
            
    # Control variables     
    header_control = ['Throttle','Rudder','Aileron','Elevator','Air breaks','High lift','Landing gear']    
        
    for i in range(len(header_control)):
        outSheet.write(0,i+len(header), header_control[i])
        
    for i in range(len(t)):
        for j in range(len(u[:,0])):
            outSheet.write(i+1,j+len(header),u[j,i])
            
    outWorkbook.close()
        
    
    
    
    
    
    
    
    
    
    