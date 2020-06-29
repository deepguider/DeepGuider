import numpy as np

gps_freq = 10   # [Hz]
wait_time = 10  # [sec]

save_file = 'Stop(%02.0fHz,%02.0fs).pose.csv' % (gps_freq, wait_time)
dt = 1. / gps_freq

data = [ [0, 0, 0, 0, 0, 0] ]
while True:
    t = data[-1][0]
    x = data[-1][1]
    y = data[-1][2]
    theta = data[-1][3]
    v = data[-1][4]
    w = data[-1][5]

    t = t + dt

    data.append([t, x, y, theta, v, w])

    if t >= wait_time: break

comment = '# Time [sec], X [m], Y [m], Theta [rad], LinVel [m/s], AngVel [rad/s]'
np.savetxt(save_file, data, fmt='%.6f', delimiter=', ', header=comment)