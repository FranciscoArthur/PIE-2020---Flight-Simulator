import numpy as np




C152_dict = {'Wing_surface': 14.79,     # [m^2]
             'Weight' : 750,            # [Kg] Aircraft weight
             'I_yy' : 897.555,          # [Kg * m^2] Moment of inertia
             'MAC': 1.454,              # [m]
             'Static_thrust': 3924,     # [N]
             'z_engine': 0,             # [m]
             'Alpha_0' : -3.49e-2,      # [rad] No lift angle of attack
        
            'Cz_Alpha' : 5.74,          # [1/rad] Lift coefficietn due to the angle of attack
            'Cz_q': 0,                  # [seg/rad] Lift coefficient due to the pitch rate - None data found in the XML file
            'Cz_Delta_m': 0.0651965,    # [1/rad] Lift due to the elevator position
            
            'Cm_0': -0.0,               # No lift pitching moment - None data found in the XML file
            'Cm_Alpha' : -5.2,          # [1/rad] Moment coefficient due to the angle of attack
            'Cm_q' : -12.4,             # [seg/rad] Moment coefficient due to the pitch rate
            'Cm_Delta_m' : -1.1,        # [1/rad] Moment coefficient due to the elevator position
            } 




def Drag_coeff(Alpha):
    
    Cx_data = [1.5, 0.1014   ,2.98E-02 ,2.16E-02 ,1.69E-02 ,1.54E-02 ,1.70E-02 ,2.23E-02 ,3.14E-02 ,6.34E-02 ,7.45E-02 ,8.68E-02 ,0.1123   ,0.1366   ,0.1595   ,0.1797   ,0.1885   ,0.1886   ,0.1852   ,0.161    ,0.1039   ,1.5]

    Alpha_data = [-1.57,-0.2793   ,-0.1396   ,-0.1047   ,-6.98E-02 ,-3.49E-02 ,0         ,3.49E-02  ,6.98E-02  ,0.1396    ,0.1571    ,0.1745    ,0.2094    ,0.2443    ,0.2793    ,0.3142    ,0.3316    ,0.3491    ,0.3665    ,0.384     ,0.4189    ,1.57]
    
    Cx = np.interp(Alpha, Alpha_data, Cx_data)
    
    return Cx
