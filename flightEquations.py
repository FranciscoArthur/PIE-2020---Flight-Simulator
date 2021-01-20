from math import sin, cos, tan

import numpy as np


def flightEquations(time, X, forces, moments, parameters):
    """
       The function flightEquations estabilish the
       nonlinear ode of the airplane 6dof equations.
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

    p_n = X[0]
    p_e = X[1]
    h = X[2]
    phi = X[3]
    theta = X[4]
    psi = X[5]
    u = X[6]
    v = X[7]
    w = X[8]
    p = X[9]
    q = X[10]
    r = X[11]

    f_x = forces[0]
    f_y = forces[1]
    f_z = forces[2]

    roll = moments[0]  # aerodynamic moment around x
    pitch = moments[1]  # aerodynamic moment around y
    yaw = moments[2]  # aerodynamic moment around z

    m = parameters.mass
    j_x = parameters.J_x
    j_y = parameters.J_y
    j_z = parameters.J_z
    j_xz = parameters.J_xz
    gamma = j_x * j_z - j_xz ** 2
    g_d = parameters.g

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
    p_n_dot = u * cos(theta) * cos(psi) + v * (-cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi)) + w * (
            sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi))
    p_e_dot = u * cos(theta) * sin(psi) + v * (cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(phi)) + w(
        -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi))
    h_dot = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta)

    # Derived state vector
    x_dot = np.array([p_n_dot, p_e_dot, h_dot, phi_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot, p_dot, q_dot, r_dot])

    return x_dot
