import numpy as np

save_file = 'Line.py.csv'
dt = 0.1    # [sec]
v_max = 1   # [m/s]
v_acc = 1   # [m/s^2]
L = 100     # [m]

data = [ [0, 0, 0, 0, 0, 0] ]
while True:
    t = data[-1][0]
    x = data[-1][1]
    y = data[-1][2]
    theta = data[-1][3]
    v = data[-1][4]
    w = data[-1][5]

    t = t + dt
    if v < v_max: v += v_acc * dt
    if v > v_max: v = v_max
    w = 0

    x = x + v * dt * np.cos(theta + w * dt / 2)
    y = y + v * dt * np.sin(theta + w * dt / 2)
    theta = theta + w * dt
    data.append([t, x, y, theta, v, w])

    if x > L: break

comment = '# Time [sec], X [m], Y [m], Theta [rad], LinVel [m/s], AngVel [rad/s]'
np.savetxt(save_file, data, fmt='%.6f', delimiter=', ', header=comment)
