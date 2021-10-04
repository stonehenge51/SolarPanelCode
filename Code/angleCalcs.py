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

def sundeclination(N):
    declination = -math.asin(0.39779 * math.cos(0.98565 * (N + 10) + 1.914 * math.sin(0.98565 * (N - 2))))
    return declination

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


def equationoftimeAcc():
    year, month, day, hour1 = greenwichmeantime()
    timezone = -7
    year = 2021
    i = 0
    daysMonth = {31,28,31,30,31,30,31,31,30,31,30,31}
    months = np.arange(1, 13, 1)
    hourscale = np.arange(1, 3, 1)
    minutescale = {1, 2}

    data = []

    for month in months:
        # numdays = daysMonth[month - 1] + 1
        numdays = 30
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
                    EoTmins = 0.00526 + EoT1 + EoT2 + EoT3 + EoT4
                    data.append(EoTmins)



    # for month in daysMonth:
    #     days = range(1, month+1)
    #     for day in days:
    #         for hour in hourscale:
    #             x1 = (367 * year) - 738567
    #             x2 = int(7 * int(year + (month - 9) / 12) / 4)
    #             x3 = int(275 * month / 9) + 9
    #             Ddays = x1 + x2 + x3 + (hour - timezone) / 24
    #
    #             Eo = 279.6296
    #             Wo = 283.2989
    #             e = 0.0167
    #             ydays = 365.2422
    #             no = 23.4364
    #             nrad = np.pi / 180 * no
    #
    #             Mo = (360 * Ddays) / ydays + Eo - Wo
    #             Mo = rangecheck(Mo)
    #             Mrad = np.pi / 180 * Mo
    #
    #             yo = Mo + Wo
    #             yo = rangecheck(yo)
    #
    #             Erad = Mrad + (e * math.sin(Mrad)) / (1 - e * math.cos(Mrad))
    #             Vrad = 2 * math.atan(np.sqrt((1 + e) / (1 - e)) * math.tan(Erad / 2))
    #             Vo = Vrad * 180 / np.pi
    #
    #             lamo = Vo + Wo
    #             lamo = rangecheck(lamo)
    #             lamrad = np.pi / 180 * lamo
    #
    #             alpharad = np.arctan2(np.sin(lamrad), np.cos(nrad))
    #             # alpharad = np.arctan(np.sin(lamrad)*np.cos(nrad)/np.cos(lamrad))
    #
    #             alphao = alpharad * 180 / np.pi
    #             alphao = rangecheck(alphao)
    #
    #             EoTo = yo - alphao
    #             if EoTo < 100:
    #                 EoTo = EoTo + 360
    #             EoTmins = 4 * EoTo
    #
    #             data.append(EoTmins)
    #             print(EoTmins)
    #             i += 1

    return data

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


timedata = equationoftimeAcc()
smoothedtimedata = smoothing(timedata, 150)

plt.plot(timedata, label="orig data")
plt.plot(smoothedtimedata, label="smoothed data")
plt.legend(fontsize = 20)
plt.grid('both', 'both')

plt.show()


