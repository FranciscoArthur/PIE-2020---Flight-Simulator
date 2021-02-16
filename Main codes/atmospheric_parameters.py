"""
Created on Wed Nov 25 12:03:06 2020

@author: Clément Gardies

Description :
Compute atmospheric properties for heights ranging from -5 km to 80 km.
The implementation is based on the ICAO standard atmosphere from 1993
All input and output are using SI units
Only single integer values are accepted as input


Inputs : plane altitude h (m)
Outputs : Atmospheric parameters at the plane altitude :
    [0] : Altitude (m)
    [1] : Air temperature (Celsius)
    [2] : Air temperature (Kelvin)
    [3] : Air pressure (Pa)
    [4] : Air density (kg/m3)
    [5] : Air speed of sound (m/s)
    [6] : Local gravitational acceleration (m/s2)
    [7] : Air dynamic viscosity (kg/m/s)
    [8] : Air kinematic viscosity (m2/s)
    [9] : Air thermal conductivity (W/m/K)


References:
===========
.. [ICAO93] International Civil Aviation Organization ; Manual Of The ICAO
            Standard Atmosphere -- 3rd Edition 1993 (Doc 7488) -- extended
            to 80 kilometres (262 500 feet)
            
.. [WISA19] Wikipedia ; International Standard Atmosphere ;
            https://en.wikipedia.org/wiki/International_Standard_Atmosphere
            Accessed: 2019-07-28
"""

from itertools import tee
import numpy as np


def atmospheric_parameters_fct(h_geometrical):  # h = altitude (m)
    if not isinstance(h_geometrical, int):
        raise ValueError("The input 'altitude' must be an integer")

    h = geom2geop_height(h_geometrical)  # Geopotential altitude [m]
    t = temperature(h)  # Temperature [K]
    p = pressure(h, t)  # Pressure [Pa]

    altitude = h_geometrical
    temp_celsius = temperature_in_celsius(t)
    temp_kelvin = t
    press = p
    dens = density(p, t)
    speed_sound = speed_of_sound(t)
    grav_acceleration = grav_accel(h_geometrical)
    dyn_viscosity = dynamic_viscosity(t)
    kin_viscosity = dyn_viscosity / dens
    th_conductivity = thermal_conductivity(t)

    return ([altitude,
             temp_celsius,
             temp_kelvin,
             press,
             dens,
             speed_sound,
             grav_acceleration,
             dyn_viscosity,
             kin_viscosity,
             th_conductivity])


### CALCULATION CONSTANTS  ###
"""
Constants defined in the ICAO standard atmosphere (1993)

Notes:
    * All values are given in SI-units.
"""

# Constants
g_0 = 9.80665  # g_0: Standard gravitational acceleration [m/s^2]
M_0 = 28.964420e-3  # M_0: Sea level molar mass [kg/mol]
N_A = 602.257e21  # N_A: Avogadro constant [1/mol]
P_0 = 101.325e3  # P_0: Sea level atmospheric pressure [Pa]
R_star = 8314.32e-3  # R_star: Universal gas constant [J/K*mol]
R = 287.05287  # R: Specific gas constant [J/K*kg]
S = 110.4  # S: Sutherland's empirical constant in the equation for dynamic viscosity [K]
T_i = 273.15  # T_i: Temperature of the ice point at mean sea level [K]
T_0 = 288.15  # T_0: Sea level temperature [K]
t_i = 0.0  # t_i: Celsius temperature of the ice point at mean sea level [degC]
t_0 = 15.0  # t_0: Celsius sea level temperature [degC]
beta_s = 1.458e-6  # beta_s: Sutherland's empirical constant in the equation for dynamic viscosity [kg/(m*s*K**(1/2))]
kappa = 1.4  # kappa: Adiabatic index [-]
rho_0 = 1.225  # rho_0: Sea level atmospheric density [kg/m^3]
sigma = 0.365e-9  # sigma: Effective collision diameter of an air molecule [m]
r = 6_356_766  # r: Nominal Earth's radius [m]
h_min = -5_004  # h_min: Lower boundary of acceptable geometric heights [m]
h_max = 81_020  # h_max: Upper boundary of acceptable geometric heights [m]
H_min = -5_000  # H_min: Lower boundary of acceptable geopotential heights [m]
H_max = 80_000  # H_max: Upper boundary of acceptable geopotential heights [m]

# LAYER_SPEC_PROP: Table containing layer specific properties
LAYER_SPEC_PROP = [
    [-5.0e3, 320.65, -6.5e-3, 1.77687e+5, 'troposphere'],
    [000000, 288.15, -6.5e-3, 1.01325e+5, 'troposphere'],
    [11.0e3, 216.65, 0.0e-3, 2.26320e+4, 'tropopause'],
    [20.0e3, 216.65, 1.0e-3, 5.47487e+3, 'stratosphere'],
    [32.0e3, 228.65, 2.8e-3, 8.68014e+2, 'stratosphere'],
    [47.0e3, 270.65, 0.0e-3, 1.10906e+2, 'stratopause'],
    [51.0e3, 270.65, -2.8e-3, 6.69384e+1, 'mesosphere'],
    [71.0e3, 214.65, -2.0e-3, 3.95639e+0, 'mesosphere'],
    [80.0e3, 196.65, -2.0e-3, 8.86272e-1, 'mesosphere'],
]
"""
Notes on 'LAYER_SPEC_PROP':
      * Table with columns
            1. :H_b: geopotential base height [m]
            2. :T_b: base temperature [K]
            3. :beta: base temperature gradient [kg/(m*s*K^(1/2))]
            4. :p: base pressure [Pa]
            5. :layer name: string representation of layer name
      * Values for (1,2,3) from table D in [ICAO93]_
      * Values for (4) for pressure from [ICAO93]_
      * Values for (5) from [WISA19]_
"""

# Number of first and last layer in atmosphere
LAYER_NUM_FIRST = 1
LAYER_NUM_LAST = len(LAYER_SPEC_PROP) - 1


# PAIRWISE: Function to build up pairs from a list
def pairwise(iterable):
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


# LAYER_DICTS: Dictionary containing layer specific properties
LAYER_DICTS = {}
MAX_STR_LEN_LAYER_NAME = 0
for i, layer_pair in enumerate(pairwise(LAYER_SPEC_PROP), start=LAYER_NUM_FIRST):
    # Layer properties from the base layer are valid from base to top
    H_base, T, beta, p, layer_name = layer_pair[0]
    H_top, _, _, _, _ = layer_pair[1]

    LAYER_DICTS[i] = {
        'H_base': H_base,
        'H_top': H_top,
        'T': T,
        'beta': beta,
        'p': p,
        'name': layer_name,
    }

    if len(layer_name) > MAX_STR_LEN_LAYER_NAME:
        MAX_STR_LEN_LAYER_NAME = len(layer_name)


### CALCULATION FUNCTIONS  ###
def _get_layer_nums(H):
    """Return array of same shape as 'H' with corresponding layer numbers"""

    layers = LAYER_DICTS
    layer_nums = 0

    if (H < H_min) or (H >= H_max):
        raise ValueError("The current altitude cannot be computed - must be between -5km and 80km")

    for i in layers.keys():
        pos_in_layer = (H >= layers[i]['H_base']) & (H < layers[i]['H_top'])
        layer_nums += int(pos_in_layer) * i
    return int(layer_nums)


def _get_layer_params(H):
    """
    Get layer specific data for given geopotential height 'H'
    Returns: (H_b, T_b, beta) layer specific data
    """

    H_b = 0
    T_b = 0
    beta = 0
    p_b = 0

    for i, layer_dict in LAYER_DICTS.items():
        pos_in_layer = int((_get_layer_nums(H) == i))

        H_b += pos_in_layer * layer_dict['H_base']
        T_b += pos_in_layer * layer_dict['T']
        beta += pos_in_layer * layer_dict['beta']
        p_b += pos_in_layer * layer_dict['p']

    return H_b, T_b, beta, p_b


def layer_name(H):
    """Get layer names as strings"""

    str_len = MAX_STR_LEN_LAYER_NAME
    layer_name = np.char.chararray(H.shape, itemsize=str_len, unicode=True)
    layer_name[:] = ''

    for i, layer_dict in LAYER_DICTS.items():
        this_layer = np.char.chararray(H.shape, itemsize=str_len, unicode=True)
        this_layer[:] = layer_dict['name']

        pos_in_layer = int((_get_layer_nums(H) == i))
        this_layer_filtered = np.char.multiply(this_layer, pos_in_layer)

        layer_name = np.char.add(layer_name, this_layer_filtered)
    print(layer_name)
    return layer_name


def temperature(H):
    """
    Air temperature in Kelvin
    """
    H_b, T_b, beta, _ = _get_layer_params(H)
    return T_b + beta * (H - H_b)


def pressure(H, T):
    H_b, T_b, beta, p_b = _get_layer_params(H)

    # Note: Pressure is computed differently for beta = 0 and for beta != 0
    # Get a vector with positions where beta is 0
    beta_zero = int(beta == 0)
    beta_nonzero = 1 - beta_zero

    pressure = 0

    # Pressure if beta == 0
    pressure_beta_zero = p_b * np.exp((-g_0 / (R * T)) * (H - H_b))
    pressure = pressure + pressure_beta_zero * beta_zero

    # Pressure if beta != 0
    exponent = np.divide(1, beta, where=beta != 0)
    pressue_beta_nozero = p_b * (1 + (beta / T_b) * (H - H_b)) ** (exponent * (-g_0 / R))

    pressure = pressure + pressue_beta_nozero * beta_nonzero
    return pressure


def geom2geop_height(h):
    return r * h / (r + h)


def temperature_in_celsius(T):
    """
    Air temperature in Celsius
    """
    return T - T_i


def grav_accel(h):
    return g_0 * (r / (r + h)) ** 2


def density(P, T):
    return P / (R * T)


def speed_of_sound(T):
    return np.sqrt(kappa * R * T)


def dynamic_viscosity(T):
    return beta_s * T ** 1.5 / (T + S)


def thermal_conductivity(T):
    return 2.648151e-3 * T ** 1.5 / (T + (245.4 * 10 ** (-12 / T)))


# # Test
# altitude = 10000
# print(atmospheric_parameters_fct(altitude))
