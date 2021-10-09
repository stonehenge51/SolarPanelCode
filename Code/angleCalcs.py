import numpy as np
import time
import matplotlib.pyplot as plt


def rangecheck(value):
    while value < 0:
            value = value + 360
    while value > 360:
            value = value - 360
            
    return value

def sundeclination(B, ans, year, month, day, hour, minute):
    if ans == 1:
        N = Ncalc(year, month, day, hour, minute)
        declination = np.arcsin(np.sin(-23.44 * np.pi / 180) * np.cos(2 * np.pi / 365.24 * (N + 10) + 2 * np.pi / np.pi * 0.0167 * np.sin(2 * np.pi / 365.24 * (N - 2))))
    else:
        N = B
        declination = np.arcsin(np.sin(-23.44 * np.pi / 180) * np.cos(2 * np.pi / 365.24 * (N + 10) + 2 * np.pi / np.pi * 0.0167 * np.sin(2 * np.pi / 365.24 * (N - 2))))

    return declination, N

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
        year = date_time[0]
        month = date_time[1]
        day = date_time[2]
        hour = date_time[3]
        minute = date_time[4]
        second = date_time[5]
        return hour + minute/60 + second/(60*60)
    else:
        return year, month, day, hour, minute, second

def equationoftime(year, month, day, hour, minute, accuracy):
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
    if accuracy == 1:
        EoTmins = 0.00526 + EoT1                            
    elif accuracy == 2:
        EoTmins = 0.00526 + EoT1 + EoT2
    elif accuracy == 3:
        EoTmins = 0.00526 + EoT1 + EoT2 + EoT3
    elif accuracy == 4:
        EoTmins = 0.00526 + EoT1 + EoT2 + EoT3 + EoT4
    else:
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
    delta, N = sundeclination(0, 1, year, month, day, hour+7, minute)
    Emin = equationoftime(year, month, day, hour, minute, 4)
    
    psiS = delta
    # print("Tgmt is ",Tgmt,"\nEmin is ",Emin ,"\ndeclination is ",psiS,"\nN is ", N)
    lambdaS = -15*(Tgmt - 12 + Emin/60) * np.pi/180
    # print("lambdaS is ", lambdaS)
    Sx = np.cos(psiS)*np.sin(lambdaS - lambdaO)
    Sy = np.cos(psiO) * np.sin(psiS) - np.sin(psiO) * np.cos(psiS) * np.cos(lambdaS - lambdaO)
    Sz = np.sin(psiO) * np.sin(psiS) + np.cos(psiO) * np.cos(psiS) * np.cos(lambdaS - lambdaO)
    S2 = np.sqrt(Sx*Sx + Sy*Sy + Sz*Sz)
    # print("S2 is ", S2)
    # print("Sx is ", Sx, "\nSy is ", Sy, "\nSz is ", Sz)
    
    Z = np.arcsin(Sz)
    ys = np.arctan2(Sx,Sy)
    
    # print("Z is ", Z, "\nys is ", ys)
#    print("sun declination is ", psiS * 180/np.pi)
    
    zenith = Z * 180/np.pi
    azimuth = ys * 180/np.pi
    if azimuth < 0.0:
        azimuth += 360
    
    return zenith, azimuth


year, month, day, hour, minute, second = greenwichmeantime(0)
Tgmt = greenwichmeantime(1)

elevation, azimuth = solarzenithelevation(year, month, day, hour, minute, Tgmt)
print("The value for Elevation is ", elevation, "\nThe value for Azimuth", azimuth)

elevationarray = []
azimutharray = []
hour = np.arange(0,24)
minute = np.arange(0,61)
for i in hour: 
    for j in minute:
        Tgmt = i + j / 60 + 7
        elevation, azimuth = solarzenithelevation(year, month, day, i, j, Tgmt)
        elevationarray.append(elevation)
        azimutharray.append(azimuth)


plt.plot(azimutharray, elevationarray)
plt.grid('both', 'both')
plt.show()

plt.plot(azimutharray)
plt.grid('both', 'both')
plt.show()


#timedata4 = equationoftimeAcc(4)
#timedata3 = equationoftimeAcc(3)
#timedata2 = equationoftimeAcc(2)
#timedata1 = equationoftimeAcc(1)
#
#plt.plot(timedata4, label="orig data acc4")
#plt.plot(timedata3, label="orig data acc3")
#plt.plot(timedata2, label="orig data acc2")
#plt.plot(timedata1, label="orig data acc1")
#plt.legend(fontsize = 10)
#plt.grid('both', 'both')
    

# yearlydec = []
#
# xline = [23,-23]
# yline = [279,279]
#
# for i in range(366):
#    dec, N = sundeclination(i, 0, year, month, day, hour, minute)
#    yearlydec.append(dec*180/np.pi)
#
# plt.plot(yearlydec, label="orig data")
# plt.plot(yline, xline, label="line")
# plt.legend(fontsize = 10)
# #plt.xlim([250, 300])
# #plt.ylim([10, -10])
# plt.grid('both', 'both')
# plt.show()


