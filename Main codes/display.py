import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d


def display_fct(result):
    """Plotting function that generates four graphs for the results variables history.


    Input:

        Dictionnary of the form:

            result = {}
            result['plane_position']                    =   plane_position
            result['plane_orientation']                 =   plane_orientation
            result['plane_speed']                       =   plane_speed
            result['plane_TAS']                         =   plane_TAS
            result['plane_angular_speed']               =   plane_angulas_speed
            result['plane_fuel_Load']                   =   plane_fuel_Load
            result['plane_mass']                        =   plane_mass
            result['plane_forces']                      =   plane_forces
            result['plane_moments']                     =   plane_moments
            result['plane_Mach']                        =   plane_Mach
            result['plane_qinf']                        =   plane_qinf
            result['time']                              =   time
            result['pilot_commands']                    =   pilot_commands
            result['plane_Load_factor']                 =   plane_Load_factor
            result['plane_data']                        =   plane_data
            result['plane_atmospheric_parameters']      =   plane_atmospheric_parameters



        """

    t = result['time']

    #      PLOT STATE HISTORY

    # Definition of the state vector

    x = np.zeros((12, len(t)))

    x[0] = result['plane_speed'][0]          # Body-axis x inertial velocity, ub, m/s
    x[1] = result['plane_speed'][1]          # Body-axis y inertial velocity, vb, m/s
    x[2] = result['plane_speed'][2]          # Body-axis z inertial velocity, wb, m/s
    x[3] = result['plane_position'][0]       # North position of center of mass WRT Earth, xe, m
    x[4] = result['plane_position'][1]       # East position of center of mass WRT Earth, ye, m
    x[5] = result['plane_position'][2]       # Negative of c.m. altitude WRT Earth, ze = -h, m
    x[6] = result['plane_angular_speed'][0]  # Body-axis roll rate, pr, rad/s
    x[7] = result['plane_angular_speed'][1]  # Body-axis pitch rate, qr, rad/s
    x[8] = result['plane_angular_speed'][2]  # Body-axis yaw rate, rr,rad/s
    x[9] = result['plane_orientation'][0]    # Roll angle of body WRT Earth, phir, rad
    x[10] = result['plane_orientation'][1]   # Pitch angle of body WRT Earth, thetar, rad
    x[11] = result['plane_orientation'][2]   # Yaw angle of body WRT Earth, psir, rad

    r2d = 180 / np.pi

    f1 = plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(t, x[0])
    plt.xlabel('Time, s'), plt.ylabel('Axial Velocity (u), m/s'), plt.grid(True)
    plt.title('Forward Body-Axis Component of Inertial Velocity, u')
    plt.subplot(2, 2, 2)
    plt.plot(t, x[1])
    plt.xlabel('Time, s'), plt.ylabel('Side Velocity (v), m/s'), plt.grid(True)
    plt.title('Side Body-Axis Component of Inertial Velocity, v')
    plt.subplot(2, 2, 3)
    plt.plot(t, x[2])
    plt.xlabel('Time, s'), plt.ylabel('Normal Velocity (w), m/s'), plt.grid(True)
    plt.title('Normal Body-Axis Component of Inertial Velocity, z')
    plt.subplot(2, 2, 4)
    plt.plot(t, x[0], t, x[1], t, x[2])
    plt.xlabel('Time, s'), plt.ylabel('u (blue), v (green), w (red), m/s'), plt.grid(True)
    plt.title('Body-Axis Component of Inertial Velocity')
    plt.legend(['Axial velocity, u', 'Side velocity, v', 'Normal velocity, w'])

    f2 = plt.figure()
    plt.subplot(3, 2, 1)
    plt.plot(t, x[3])
    plt.xlabel('Time, s'), plt.ylabel('North (x), m'), plt.grid(True)
    plt.title('North Location, x')

    plt.subplot(3, 2, 2)
    plt.plot(t, x[4])
    plt.xlabel('Time, s'), plt.ylabel('East (y), m'), plt.grid(True)
    plt.title('East Location, y')

    plt.subplot(3, 2, 3)
    plt.plot(t, -x[5])
    plt.xlabel('Time, s'), plt.ylabel('Altitude (-z), m'), plt.grid(True)
    plt.title('Altitude, -z')

    plt.subplot(3, 2, 4)
    plt.plot((np.sqrt(x[3] * x[3] + x[4] * x[4])), -x[5])
    plt.xlabel('Ground Range, m'), plt.ylabel('Altitude, m'), plt.grid(True)
    plt.title('Altitude vs. Ground Range')

    plt.subplot(3, 2, 5)
    plt.plot(x[3], x[4])
    plt.xlabel('North, m'), plt.ylabel('East, m'), plt.grid(True)
    plt.title('Ground Track, North vs. East')

    ax = f2.add_subplot(3, 2, 6, projection="3d")
    ax.plot3D(x[3], x[4], -x[5])
    ax.set_xlabel('North, m'), ax.set_ylabel('East, m'), ax.set_zlabel('Altitude, m'), plt.grid(True)
    ax.set_title('3D Flight Path')

    f3 = plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(t, x[6] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('Roll Rate (p), deg/s'), plt.grid(True)
    plt.title('Body-Axis Roll Component of Inertial Rate, p')
    plt.subplot(2, 2, 2)
    plt.plot(t, x[7] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('Pitch Rate (q), deg/s'), plt.grid(True)
    plt.title('Body-Axis Pitch Component of Inertial Rate, q')
    plt.subplot(2, 2, 3)
    plt.plot(t, x[8] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('Yaw Rate (r), deg/s'), plt.grid(True)
    plt.title('Body-Axis Yaw Component of Inertial Rate, r')
    plt.subplot(2, 2, 4)
    plt.plot(t, x[6] * r2d, t, x[7] * r2d, t, x[8] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('p (blue), q (green), r (red), deg/s'), plt.grid(True)
    plt.title('Body-Axis Inertial Rate Vector Components')
    plt.legend(['Roll rate, p', 'Pitch rate, q', 'Yaw rate, r'])

    f4 = plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(t, x[9] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('Roll Angle (phi), deg'), plt.grid(True)
    plt.title('Earth-Relative Roll Attitude')
    plt.subplot(2, 2, 2)
    plt.plot(t, x[10] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('Pitch Angle (theta), deg'), plt.grid(True)
    plt.title('Earth-Relative Pitch Attitude')

    plt.subplot(2, 2, 3)
    plt.plot(t, x[11] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('Yaw Angle (psi, deg'), plt.grid(True)
    plt.title('Earth-Relative Yaw Attitude')
    plt.subplot(2, 2, 4)
    plt.plot(t, x[9] * r2d, t, x[10] * r2d, t, x[11] * r2d)
    plt.xlabel('Time, s'), plt.ylabel('phi (blue), theta (green), psi (red), deg'), plt.grid(True)
    plt.title('Euler Angles')
    plt.legend(['Roll angle, phi', 'Pitch angle, theta', 'Yaw angle, psi'])

    #      PLOT CONTROL HISTORY

    # Definition of the control vector

    u = np.zeros((7, len(t)))

    u[0] = result['pilot_commands'][0]  # Elevator, dEr, rad, positive: trailing edge down
    u[1] = result['pilot_commands'][1]  # Aileron, dAr, rad, positive: left trailing edge down
    u[2] = result['pilot_commands'][2]  # Rudder, dRr, rad, positive: trailing edge left
    u[3] = result['pilot_commands'][3]  # Throttle, dT, %
    u[4] = result['pilot_commands'][4]  # Asymmetric Spoiler, dASr, rad
    u[5] = result['pilot_commands'][5]  # Flap, dFr, rad
    u[6] = result['pilot_commands'][6]  # Stabilator, dSr, rad

    plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(t, u[0], t, u[6])
    plt.xlabel('Time, s'), plt.ylabel('Elevator (blue), Stabilator (green), deg'), plt.grid(True)
    plt.title('Pitch Test Inputs'), plt.legend(['Elevator, dE', 'Stabilator, dS'])
    plt.subplot(2, 2, 2)
    plt.plot(t, u[1], t, u[2], t, u[4])
    plt.xlabel('Time, s'), plt.ylabel('Aileron (blue), Rudder (green), Asymmetric Spoiler (red), deg'), plt.grid(True)
    plt.title('Lateral-Directional Test Inputs'), plt.legend(['Aileron, dA', 'Rudder, dR', 'Asymmetric Spoiler, dAS'])
    plt.subplot(2, 2, 3)
    plt.plot(t, u[3])
    plt.xlabel('Time, s'), plt.ylabel('Throttle Setting'), plt.grid(True)
    plt.title('Throttle Test Inputs')
    plt.subplot(2, 2, 4)
    plt.plot(t, u[5])
    plt.xlabel('Time, s'), plt.ylabel('Flap, deg'), plt.grid(True)
    plt.title('Flap Test Inputs')

    #       PLOT SPEED HISTORY

    # Definition of speed variables

    tas = result['plane_TAS']  # Speed module wrt the air, consideering the wind

    mach = result['plane_Mach']  # Mach number

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t, tas)
    plt.xlabel('Time, s'), plt.ylabel('TAS'), plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(t, mach)
    plt.xlabel('Time, s'), plt.ylabel('Mach'), plt.grid(True)

    #       PLOT MASS HISTORY

    mass = result['plane_mass']  # Plane mass

    plt.figure()
    plt.plot(t, mass)
    plt.xlabel('Time, s'), plt.ylabel('Plane Mass'), plt.grid(True)

    #       PLOT LOAD FACTOR HISTORY

    nz = result['plane_Load_factor']  # Plane load factor

    plt.figure()
    plt.plot(t, nz)
    plt.xlabel('Time, s'), plt.ylabel('Load Factor'), plt.grid(True)
