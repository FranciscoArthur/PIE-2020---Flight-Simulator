from math import sqrt, asin, acos, sin, cos, pi, atan2

from scipy.optimize import fsolve

R = 6.378138 * 10 ** 8
f = 0.003353


def wgs2xyz(lat, long):
    global R
    global f

    latRad = lat * pi / 180
    longRad = long * pi / 180

    theta = fsolve(lambda thetaIt: latRad - thetaIt - f * sin(2 * thetaIt), latRad)
    theta = theta[0]

    Re = R * (1 + (1 / (1 - f) ** 2 - 1) * sin(theta)) ** (-1 / 2)

    x = Re * cos(theta) * cos(longRad)
    y = Re * cos(theta) * sin(longRad)
    z = Re * sin(theta)

    return x, y, z


def xyz2wgs(x, y, z):
    global R
    global f

    Re = sqrt(x ** 2 + y ** 2 + z ** 2)

    theta = asin(z / Re)
    lat = f * sin(2 * theta) + theta
    long = acos(round(x / (Re * cos(theta)), 10))

    if y < 0:
        long = -long

    return lat * 180 / pi, long * 180 / pi


def flat2lla(flat, llo, psiRef, hRef):
    global R
    global f

    xp = flat[0]
    yp = flat[1]
    zp = flat[2]

    muRef = llo[0]
    lRef = llo[1]

    muRef = muRef * pi / 180
    lRef = lRef * pi / 180
    psiRef = psiRef * pi / 180

    dN = xp * cos(psiRef) - yp * psiRef
    dE = xp * sin(psiRef) + yp * cos(psiRef)

    Rn = R / sqrt(1 - (2 * f - f ** 2) * sin(muRef) ** 2)
    Rm = Rn * (1 - (2 * f - f ** 2)) / (1 - (2 * f - f ** 2) * sin(muRef) ** 2)

    dMu = atan2(1, Rm) * dN
    dL = atan2(1, Rn * cos(muRef)) * dE

    mu = muRef + dMu
    L = lRef + dL
    h = -zp - hRef

    return mu * 180 / pi, L * 180 / pi, h


def lla2flat(lla, llo, psio, href):
    global R
    global f
    '''
    from:https://gist.github.com/skakri/ca94fd73bf7483c1fffec500c92cacf8
    lla  -- array of geodetic coordinates
            (latitude, longitude, and altitude),
            in [degrees, degrees, meters].
            Latitude and longitude values can be any value.
            However, latitude values of +90 and -90 may return
            unexpected values because of singularity at the poles.
    llo  -- Reference location, in degrees, of latitude and
            longitude, for the origin of the estimation and
            the origin of the flat Earth coordinate system.
    psio -- Angular direction of flat Earth x-axis
            (degrees clockwise from north), which is the angle
            in degrees used for converting flat Earth x and y
            coordinates to the North and East coordinates.
    href -- Reference height from the surface of the Earth to
            the flat Earth frame with regard to the flat Earth
            frame, in meters.
    usage: print(lla2flat((0.1, 44.95, 1000.0), (0.0, 45.0), 5.0, -100.0))
    '''

    Lat_p = lla[0] * pi / 180.0  # from degrees to radians
    Lon_p = lla[1] * pi / 180.0  # from degrees to radians
    Alt_p = lla[2]  # meters

    # Reference location (lat, lon), from degrees to radians
    Lat_o = llo[0] * pi / 180.0
    Lon_o = llo[1] * pi / 180.0

    psio = psio * pi / 180.0  # from degrees to radians

    dLat = Lat_p - Lat_o
    dLon = Lon_p - Lon_o

    ff = (2.0 * f) - (f ** 2)  # Can be precomputed

    sinLat = sin(Lat_o)

    # Radius of curvature in the prime vertical
    Rn = R / sqrt(1 - (ff * (sinLat ** 2)))

    # Radius of curvature in the meridian
    Rm = Rn * ((1 - ff) / (1 - (ff * (sinLat ** 2))))

    dNorth = (dLat) / atan2(1, Rm)
    dEast = (dLon) / atan2(1, (Rn * cos(Lat_o)))

    # Rotate matrice clockwise
    Xp = (dNorth * cos(psio)) + (dEast * sin(psio))
    Yp = (-dNorth * sin(psio)) + (dEast * cos(psio))
    Zp = -Alt_p - href

    return Xp, -Yp, Zp


if __name__ == '__main__':
    lat = -37
    long = 101.7210

    x, y, z = wgs2xyz(lat, long)
    lat2, long2 = xyz2wgs(x, y, z)

    print('x , y , z = ' + str(x) + ', ' + str(y) + ', ' + str(z))
    print('latOrig = ' + str(lat))
    print('lat2 = ' + str(lat2))

    print('LongOrig = ' + str(long))
    print('Long2 = ' + str(long2))

    # for latI in range(-90, 90):
    #     for longJ in range(-179, 180):
    #         x, y, z = wgs2xyz(latI, longJ)
    #         lat2, long2 = xyz2wgsV2(x, y, z)
    #         print('i = ' + str(latI) + ', j = ' + str(longJ))
    #         if (latI - lat2)>1 or latI-lat2<-1 or longJ-long2>1 or longJ-long2<-1:
    #             print('-----------------------')
    #             print('latOrig = ' + str(latI))
    #             print('lat = ' + str(lat2))
    #             print('LongOrig = ' + str(longJ))
    #             print('Long = ' + str(long2))
    #
    print(flat2lla([x, y, z], [0, 10], 0, -1000))
