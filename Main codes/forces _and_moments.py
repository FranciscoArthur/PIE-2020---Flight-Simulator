import math
import numpy as np


# Thrust
def thrust_fct(rho, D, n, C_t):  # D = diameter and n = rotation speed
    return rho * C_t * (n ** 2) * (D ** 4)


# Function
def forces_and_moments(coeffs_aero, plane_data, plane_speed, atmosphere_data, plane_orientation):
    g = 9.81

    rho = atmosphere_data[4]

    S = plane_data[0]
    b = plane_data[1]  # b = wingspan
    c = plane_data[2]  # c = mean aerodynamic chord
    l = plane_data[3]  # l = mean aerodynamic chord of VTP
    m = plane_data[4]
    mf = plane_data[5] # mf = fuel mass

    v = math.sqrt(plane_speed[0] ** 2 + plane_speed[1] ** 2 + plane_speed[2] ** 2)

    cz = coeffs_aero[0]
    cx = coeffs_aero[1]
    cy = coeffs_aero[2]
    cl = coeffs_aero[3]
    cm = coeffs_aero[4]
    cn = coeffs_aero[5]

    theta = plane_orientation[0]
    psi = plane_orientation[1]
    phi = plane_orientation[2]

    drag = 0.5 * rho * S * cx * (v ** 2)

    lat_force = 0.5 * rho * S * cy * (v ** 2)

    weight = m * g

    lift = 0.5 * rho * S * cz * (v ** 2)

    roll_m = 0.5 * rho * S * b * cl * (v ** 2)

    pitch_m = 0.5 * rho * S * c * cm * (v ** 2)

    yaw_m = 0.5 * rho * S * l * cn * (v ** 2)

    thrust = 0  # TODO: add a correct model for the thrust

    f_x = thrust - drag - weight * math.sin(theta)

    f_y = lat_force + weight * math.sin(phi) * np.sign(phi)

    f_z = lift - weight * math.cos(theta)

    return [f_x, f_y, f_z, pitch_m, roll_m, yaw_m]
