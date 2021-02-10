from math import sin, cos, tan

import numpy as np


def flight_equations(time, state_vector, forces, moments, parameters):
    """
       The function flight_equations establish the
       nonlinear ODE of the airplane 6dof equations.
       It uses the state vector as input with forces
       forces and moments to generate de derived
       state vector to be solved with a iterative
       ODE solving method, as runge-kutta.

       Inputs: Time, State vector, airplane forces,
             airplane moments, airplane parameters.
       Outputs: Derived state vector

       Author: Francisco Arthur Bonfim Azevedo
       Date: 20/01/2021

    """
    u = state_vector[0]       # Body axis x inertial velocity in m/s
    v = state_vector[1]       # Body axis y inertial velocity in m/s
    w = state_vector[2]       # Body axis z inertial velocity in m/s
    x = state_vector[3]       # North position of center of mass wrt earth in m
    y = state_vector[4]       # East position of center of mass wrt earth in m
    z = state_vector[5]       # Negative of center of mass altitude wrt earth ( z = - h ) in m
    p = state_vector[6]       # Body-axis roll rate in rad/s
    q = state_vector[7]       # Body-axis pitch rate in rad/s
    r = state_vector[8]       # Body-axis yaw rate in rad/s
    phi = state_vector[9]    # Roll angle of body wrt earth in rad
    theta = state_vector[10]  # Roll angle of body wrt earth in rad
    psi = state_vector[11]    # Roll angle of body wrt earth in rad

    f_x = forces[0]           # Forces acting on the airplane on the x axis (body axis)
    f_y = forces[1]           # Forces acting on the airplane on the y axis (body axis)
    f_z = forces[2]           # Forces acting on the airplane on the z axis (body axis)

    roll = moments[0]         # Moments acting on the airplane on the x axis (body axis)
    pitch = moments[1]        # Moments acting on the airplane on the y axis (body axis)
    yaw = moments[2]          # Moments acting on the airplane on the z axis (body axis)

    m = parameters.mass       # Airplane's mass
    j_x = parameters.J_x      # Airplane's moment of inertia along x (body axis)
    j_y = parameters.J_y      # Airplane's moment of inertia along y (body axis)
    j_z = parameters.J_z      # Airplane's moment of inertia along z (body axis)
    j_xz = parameters.J_xz    # Airplane's crossed moment of inertia of x and z (body axis)
    g_d = parameters.g        # Earth's gravity (Earth's axis)

    gamma = j_x * j_z - j_xz ** 2

    # Force Equation
    u_dot = r * v - q * w - g_d * sin(theta) + f_x / m
    v_dot = -r * u + p * w + g_d + sin(phi) * cos(theta) + f_y / m
    w_dot = q * u - p * v + g_d * cos(phi) * cos(theta) + f_z / m

    # Kinematic Equations
    phi_dot = p + tan(theta) * (q * sin(phi) + r * cos(phi))
    theta_dot = q * cos(phi) - r * sin(phi)
    psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta)

    # Moment Equations
    p_dot = j_xz * (j_x - j_y + j_z) * p * q - (j_z * (j_z - j_y) + j_xz ** 2) * q * r + j_z * roll + j_xz * yaw / gamma
    q_dot = ((j_z - j_x) * p * r - j_xz * (p ** 2 - r ** 2) + pitch) / j_y
    r_dot = ((j_x - j_y) * j_x + j_xz ** 2) * p * q - j_xz * (j_x - j_y + j_z) * q * r + j_xz * roll + j_x * yaw / gamma

    # Navigation Equations
    xe_dot = u * cos(theta) * cos(psi) + v * (-cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi)) + w * (
            sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi))
    ye_dot = u * cos(theta) * sin(psi) + v * (cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(phi)) + w(
        -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi))
    ze_dot = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta)

    # Derived state vector
    x_dot = np.array([u_dot, v_dot, w_dot, xe_dot, ye_dot, ze_dot,  p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot])

    return x_dot
