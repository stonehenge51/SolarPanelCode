import numpy as np
import math
import time
import matplotlib.pyplot as plt


def sinc(length):  # declares my sinc function kernel
    size = length  # number of data points
    stepf = [0] * size  # holds the sinc function values
    total = 0.0  # summing variable to create normalized kernel
    odd = 0  # for adjusting the upper point if the length is even
    fc = 0.01

    if size % 2 == 0:  # if the length required is even subtract one to the upper limit
        odd = -1  # set odd value to negative one

    lower = -int(length / 2)  # defines the lower boundary of the kernel function
    upper = int(length / 2) + odd  # defines the upper boundary of the kernel function

    for i in range(lower, upper):  # this for loop calculates the the kernel values
        if i != 0:  # this is for leaving out i = 0 since sin(0)/0 is undefined
            stepf[i + upper] = 2*fc*(np.sin(2*fc*np.pi*i) / (2*fc*np.pi*i))  # assigns the sinc function values to to the kernel array
            total += stepf[i + upper]  # sums the values of stepf to normalize the heights

        else:  # if i = 0 use this close to zero calculation
            num1 = 1e-15  # close to zero number
            stepf[i + upper] = np.sin(i + num1) / num1  # approaching zero sinc function calculation
            total += stepf[i + upper]  # sums the values of stepf to normalize the heights

    for i in range(0, length):  # this for loop normalizes the kernel values
        stepf[i] /= total  # this calculation normalizes the heights of the kernel values

    return stepf  # returns the the sinc function filter kernel

def smoothing(data, filterlength):
    if (filterlength % 2) == 1:
        sinclength = int(filterlength/2)
        zerolength = sinclength + 1
    else:
        sinclength = int(filterlength / 2)
        zerolength = sinclength

    zeros = [0.0] * zerolength
    sincfilter = sinc(sinclength) + zeros
    fftsincfilter = np.fft.fft(sincfilter)

    rows = int(len(data) / sinclength)
    cols = filterlength
    matrix = []
    for i in range(rows):
        row = []
        for j in range(cols):
            if j < sinclength:
                row.append(data[i * sinclength + j])
            else:
                row.append(0)

        fftseg = np.fft.fft(row)
        matrix.append(fftseg)

    output = []

    for i in range(rows):
        fftrow = matrix[i]
        outputrow = []

        for j in range(cols):
            outputrow.append(fftrow[j] * fftsincfilter[j])

        outseg = np.fft.ifft(outputrow)
        output.append(outseg)

    outputtotal = [0.0] * (len(data) + filterlength)

    for i in range(rows):
        row = output[i]

        for j in range(cols):
            if i == 0:
                outputtotal[j] = row[j]

            if i != 0:
                outputtotal[i * sinclength + j] = outputtotal[i * sinclength + j] + row[j]

    return outputtotal

def sundeclination(N, ans):
    if ans == 1:
        N = Ncalc()
    declination = np.arcsin(np.sin(-23.44*np.pi/180) * np.cos(2*np.pi/365.24 * (N + 10) + 2*np.pi/np.pi * 0.0167 * np.sin(2*np.pi/365.24 * (N - 2))))
    return declination, N

def greenwichmeantime():
    date_time = time.localtime()
    year = date_time[0]
    month = date_time[1]
    day = date_time[2]
    hour = date_time[3]

    return year, month, day, hour

def rangecheck(value):
    # print("starting value ", value)
    while value < 0:
            value = value + 360
    while value > 360:
            value = value - 360

    # print("ending value ", value)
    return value


def equationoftimeAcc(accuracy):
    year, month, day, hour1 = greenwichmeantime()
    timezone = -7
    year = 2021
    daysMonth = [31,28,31,30,31,30,31,31,30,31,30,31]
    months = np.arange(1, 13, 1)
    hourscale = np.arange(1, 3, 1)
    minutescale = [1, 2]

    data = []

    for month in months:
        numdays = daysMonth[month-1]
        days = range(1, numdays)
        for day in days:
            for hour in hourscale:
                for minute in minutescale:

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
                            
                    data.append(EoTmins)

    return data

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

def Ncalc():
    daysMonth = [31,28,31,30,31,30,31,31,30,31,30,31]
    days = 0
    
    t = time.gmtime()
    year = t[0]
    month = t[1]
    day = t[2]
    hour = t[3]
    minute = t[4]
    print("month is ", month)
    
    for i in range(month-1):
        if year%4 != 0:
            days = days + daysMonth[i]
        else:
            if i == 1:
                days = days + daysMonth[i] + 1
    
    
    return days + day - 7/24

def solarzenithelevation():
    zenith = 0
    elevation = 0
    psiS, N = sundeclination(0, 1)
    declination = -23.45 * np.cos(2*np.pi/365*(N+10))
    print("N is ", N)
    print("sun declination is ", declination)
    print("sun declination is ", psiS * 180/np.pi)
    
    return zenith, elevation



num1, num2 = solarzenithelevation()
    

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
    

yearlydec = []

xline = [23,-23]
yline = [279,279]

for i in range(366):
    dec, N = sundeclination(i, 0)
    yearlydec.append(dec*180/np.pi)
    
plt.plot(yearlydec, label="orig data")
plt.plot(yline, xline, label="line")
plt.legend(fontsize = 10)
#plt.xlim([250, 300])
#plt.ylim([10, -10])
plt.grid('both', 'both')
plt.show()


