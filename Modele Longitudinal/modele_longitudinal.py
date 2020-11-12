from rk4 import rk4
from ambiance import Atmosphere
import C152_data
import numpy as np
import matplotlib.pyplot as plt

Plane_data = C152_data.C152_dict

g=9.81

def dX (t, X):
    
    # State variables definition
    V = X[0]        # TAS
    Gamma = X[1]    # Flight path angle
    Alpha = X[2]    # Angle of attack
    q = X[3]        # Pitch rate
    h = X[4]        # Altitude
    
    
    # Control varaibles definition
    Delta_x = 0.3                     # Thrust varible        
    Delta_m = -10*(np.pi/180)         # Elevator variable
    
    rho = Atmosphere(0).density[0]
    Q = 0.5 * rho * V**2
    Cm = Plane_data['Cm_0'] + Plane_data['Cm_Alpha']*(Alpha-Plane_data['Alpha_0']) + Plane_data['Cm_q'] * (q*Plane_data['MAC']/V) + Plane_data['Cm_Delta_m'] * Delta_m
    Cz = Plane_data['Cz_Alpha'] * (Alpha-Plane_data['Alpha_0']) + Plane_data['Cz_q'] * (q*Plane_data['MAC']/V) + Plane_data['Cz_Delta_m'] * Delta_m
    Cx = C152_data.Drag_coeff(Alpha)
    F = Plane_data['Static_thrust'] * Delta_x



    dV = (-Q * Plane_data['Wing_surface'] * Cx + F - Plane_data['Weight'] * g * np.sin(Gamma))*(1.0/Plane_data['Weight'])
    dGamma = (-Q * Plane_data['Wing_surface'] * Cz + Plane_data['Weight']*g*np.cos(Gamma))*(-1.0/(Plane_data['Weight']*V))
    dAlpha = q - dGamma
    dq = (Q * Plane_data['Wing_surface'] * Plane_data['MAC'] * Cm + F * Plane_data['z_engine'])*(1.0/Plane_data['I_yy'])
    dh = V * np.sin(Gamma)
    
    dX = np.array([dV, dGamma, dAlpha, dq, dh])
    return dX


X0 = [40.0, 0.0*(np.pi/180), 3.0*(np.pi/180), 0.0, 1000.0]      #Initial conditions



# Simulation parameters

initial_time = 0
final_time = 50
tspan = [initial_time,final_time]

n = (final_time-initial_time)*30

X = rk4(dX,tspan,X0,n)  # Solving the equations with Runge-Kutta method


# Graphiques

fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1)
fig.suptitle('State variables evolution in time')
         
ax1.plot(X[0],X[1][:,0])
ax1.set_ylabel('V')
ax1.grid(True)
         
ax2.plot(X[0],X[1][:,1]* (180/np.pi))
ax2.set_ylabel('Gamma [deg]')
ax2.grid(True)
         
ax3.plot(X[0],X[1][:,2]* (180/np.pi))
ax3.set_ylabel('Alpha [deg]')
ax3.grid(True)
         
ax4.plot(X[0],X[1][:,3]* (180/np.pi))
ax4.set_ylabel('q [deg/seg]')
ax4.grid(True)

ax5.plot(X[0],X[1][:,4])
ax5.set_ylabel('h')
ax5.set_xlabel('Time [seg]')
ax5.grid(True)

plt.show()

    
        





