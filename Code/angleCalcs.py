import numpy as np
import time
import matplotlib.pyplot as plt


def rangecheck(value):
    while value < 0:
            value = value + 360
    while value > 360:
            value = value - 360
            
    return value

def sundeclination(year, month, day, hour, minute):
    N = Ncalc(year, month, day, hour, minute)
    declination = np.arcsin(np.sin(-23.44 * np.pi / 180) * np.cos(2 * np.pi / 365.24 * (N + 10) + 2 * np.pi / np.pi * 0.0167 * np.sin(2 * np.pi / 365.24 * (N - 2))))

    return declination

def greenwichmeantime(switchstate):
    date_time = time.localtime()
    year = date_time[0]
    month = date_time[1]
    day = date_time[2]
    hour = date_time[3]
    minute = date_time[4]
    second = date_time[5]

    if switchstate == 1:
        date_time = time.gmtime()
        hour = date_time[3]
        minute = date_time[4]
        second = date_time[5]
        return hour + minute/60 + second/(60*60)
    else:
        return year, month, day, hour, minute, second

def equationoftime(year, month, day, hour, minute):
    timezone = -7
    
    aaa = 367 * year - 730531.5
    bbb = -int((7 * int(year + (month + 9)/12))/4)
    ccc = int(275 * month/9) + day
    Dtoday = (hour + minute/60 - timezone)/24
    Ddays = aaa + bbb + ccc + Dtoday
    Cycle = int(Ddays / 365.25)
    thetarad = 0.0172024 * (Ddays - 365.25 * Cycle)
    amp1 = 7.36303 - Cycle * 0.00009
    amp2 = 9.92465 - Cycle * 0.00014
    phi1 = 3.07892 - Cycle * 0.00019
    phi2 = -1.38995 + Cycle * 0.00013
    EoT1 = amp1 * np.sin(1 * (thetarad + phi1))
    EoT2 = amp2 * np.sin(2 * (thetarad + phi2))
    EoT3 = 0.31730 * np.sin(3 * (thetarad - 0.94686))
    EoT4 = 0.21922 * np.sin(4 * (thetarad - 0.60716))
    EoTmins = 0.00526 + EoT1 + EoT2 + EoT3 + EoT4

    return EoTmins

def Ncalc(year, month, day, hour, minute):
    daysMonth = [31,28,31,30,31,30,31,31,30,31,30,31]
    days = 0
    
    for i in range(month-1):
        if year%4 != 0:
            days = days + daysMonth[i]
        else:
            if i == 1:
                days = days + daysMonth[i] + 1
    
    return days + day + hour/24 + minute/1440

def solarzenithelevation(year, month, day, hour, minute, Tgmt):
    lambdaO = -123.12722630891494 * np.pi/180
    psiO = 49.17491793381123 * np.pi/180
    delta = sundeclination(year, month, day, hour+7, minute)
    Emin = equationoftime(year, month, day, hour, minute)
    
    psiS = delta
    lambdaS = -15*(Tgmt - 12 + Emin/60) * np.pi/180
    Sx = np.cos(psiS)*np.sin(lambdaS - lambdaO)
    Sy = np.cos(psiO) * np.sin(psiS) - np.sin(psiO) * np.cos(psiS) * np.cos(lambdaS - lambdaO)
    Sz = np.sin(psiO) * np.sin(psiS) + np.cos(psiO) * np.cos(psiS) * np.cos(lambdaS - lambdaO)
    
    Z = np.arcsin(Sz)
    ys = np.arctan2(Sx,Sy)

    zenith = Z * 180/np.pi
    azimuth = ys * 180/np.pi
    if azimuth < 0.0:
        azimuth += 360
    
    return zenith, azimuth


year, month, day, hour, minute, second = greenwichmeantime(0)
print(year," ", month," ", day," ", hour," ", minute," ", second)
Tgmt = greenwichmeantime(1)

elevation, azimuth = solarzenithelevation(year, month, day, hour, minute, Tgmt)
print("The value for Elevation is ", elevation, "\nThe value for Azimuth", azimuth)

