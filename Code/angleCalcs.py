import numpy as np
import math
import time
import matplotlib.pyplot as plt

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
    timezone = 7
    i = 0
    hourscale = np.arange(0.1, 24, 0.1)
    data = np.zeros(len(hourscale))

    for hour in hourscale:
        x1 = (367 * year) - 738567
        x2 = int(7 * int(year + (month - 9) / 12) / 4)
        x3 = int(275 * month / 9) + 9
        Ddays = x1 + x2 + x3 + (hour - timezone) / 24

        Eo = 279.6296
        Wo = 283.2989
        e = 0.0167
        ydays = 365.2422
        no = 23.4364
        nrad = np.pi / 180 * no

        Mo = (360 * Ddays) / ydays + Eo - Wo
        Mo = rangecheck(Mo)
        Mrad = np.pi / 180 * Mo

        yo = Mo + Wo
        yo = rangecheck(yo)

        Erad = Mrad + (e * math.sin(Mrad)) / (1 - e * math.cos(Mrad))
        Vrad = 2 * math.atan(np.sqrt((1 + e) / (1 - e)) * math.tan(Erad / 2))
        Vo = Vrad * 180 / np.pi

        lamo = Vo + Wo
        lamo = rangecheck(lamo)
        lamrad = np.pi / 180 * lamo

        alpharad = np.arctan2(lamrad, nrad)

        alphao = alpharad * 180 / np.pi
        alphao = rangecheck(alphao)
        #     alphah = alphao / 15

        #     deltarad = arcsin(sin(lamrad) * sin(nrad))
        #     deltao = deltarad * 180/pi()

        EoTo = yo - alphao
        EoTmins = 4 * EoTo

        data[i] = EoTmins
        #         print(EoTmins)
        i += 1

    return data, hourscale


timedata, hourscale = equationoftimeAcc()

plt.plot(hourscale, timedata)
plt.show()


