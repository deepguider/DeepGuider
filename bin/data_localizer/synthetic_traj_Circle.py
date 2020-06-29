import numpy as np

gps_freq = 10       # [Hz]
sim_freq = 100      # [Hz]
wait_time = 0       # [sec]
v_max = 1           # [m/s]
v_acc = 1           # [m/s^2]
R = 10              # [m]

save_file = 'Circle(%02.0fHz,%02.0fs).pose.csv' % (gps_freq, wait_time)
dt = 1. / sim_freq

data = [ [0, 0, 0, 0, 0, 0] ]
while True:
    t = data[-1][0]
    x = data[-1][1]
    y = data[-1][2]
    theta = data[-1][3]
    v = data[-1][4]
    w = data[-1][5]

    for i in range(int(sim_freq / gps_freq)):
        t = t + dt
        if t >= wait_time:
            if v < v_max: v += v_acc * dt
            if v > v_max: v = v_max
            w = v / R

        x = x + v * dt * np.cos(theta + w * dt / 2)
        y = y + v * dt * np.sin(theta + w * dt / 2)
        theta = theta + w * dt

    data.append([t, x, y, theta, v, w])

    if theta > 2 * np.pi: break

comment = '# Time [sec], X [m], Y [m], Theta [rad], LinVel [m/s], AngVel [rad/s]'
np.savetxt(save_file, data, fmt='%.6f', delimiter=', ', header=comment)
