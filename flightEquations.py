from math import sin, cos, tan

import numpy as np


def flightEquations(time, X, forces, parameters):
    ''' Small function explanation'''

    p_n = X[0]
    p_e = X[1]
    h = X[2]
    phi = X[3]
    theta = X[4]
    psi = X[5]
    u = X[6]
    v = X[7]
    w = X[8]
    P = X[9]
    Q = X[10]
    R = X[11]

    F_x = forces[0]
    F_y = forces[1]
    F_z = forces[2]

    m = parameters.mass
    J_x = parameters.J_x
    J_y = parameters.J_y
    J_z = parameters.J_z
    J_xz = parameters.J_xz
    Gama = J_x * J_z - J_xz ** 2
    g_d = parameters.g
    l = parameters.l
    n = parameters.n

    # Force Equation
    u_dot = R * v - Q * w - g_d * sin(theta) + F_x / m
    v_dot = -R * u + P * w + g_d + sin(phi) * cos(theta) + F_y / m
    w_dot = Q * u - P * v + g_d * cos(phi) * cos(theta) + F_z / m

    # Kinematic Equations
    phi_dot = P + tan(theta) * (Q * sin(phi) + R * cos(phi))
    theta_dot = Q * cos(phi) - R * sin(phi)
    psi_dot = (Q * sin(phi) + R * cos(phi)) / cos(theta)

    # Moment Equations
    P_dot = J_xz * (J_x - J_y + J_z) * P * Q - (J_z * (J_z - J_y) + J_xz ** 2) * Q * R + J_z * l + J_xz * n / Gama
    Q_dot = ((J_z - J_x) * P * R - J_xz * (P ** 2 - R ** 2) + m) / J_y
    R_dot = ((J_x - J_y) * J_x + J_xz ** 2) * P * Q - J_xz * (J_x - J_y + J_z) * Q * R + J_xz * l + J_x * n

    # Navigation Equations
    p_n_dot = u * cos(theta) * cos(psi) + v * (-cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi)) + w * (
            sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi))
    p_e_dot = u * cos(theta) * sin(psi) + v * (cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(phi)) + w(
        -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi))
    h_dot = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta)

    # Derived state vector
    X_dot = np.array([p_n_dot, p_e_dot, h_dot, phi_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot, P_dot, Q_dot, R_dot])

    return X_dot
