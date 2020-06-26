import numpy as np
import matplotlib.pyplot as plt

gps_freq = 10               # [Hz]
sim_freq = 100              # [Hz]
wait_time = 0               # [sec]
v_max = 1                   # [m/s]
v_acc = 1                   # [m/s^2]
w_max = 90 * np.pi / 180    # [rad/s]
w_acc = 90 * np.pi / 180    # [rad/s^2]
box_width = 10              # [m]
box_num = 4

save_file = 'Square(%02.0fHz,%02.0fs).pose.csv' % (gps_freq, wait_time)
dt = 1. / sim_freq
v_stop = v_max ** 2 / 2 / v_acc
w_stop = w_max ** 2 / 2 / w_acc
state = 0
cycle = 1
v_ref = v_max
w_ref = 0

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
            if v < v_ref:
                v += v_acc * dt
                if v > v_ref: v = v_ref
            if v > v_ref:
                v -= v_acc * dt
                if v < v_ref: v = v_ref
            if w < w_ref:
                w += w_acc * dt
                if w > w_ref: w = w_ref
            if w > w_ref:
                w -= w_acc * dt
                if w < w_ref: w = w_ref

            if state == 0 and (x >= (2 * cycle - 1) * box_width - v_stop):
                state = 1
                v_ref = 0
                w_ref = 0
            elif state == 1 and (v <= 0):
                state = 2
                v_ref = 0
                w_ref = w_max
            elif state == 2 and (theta >= np.pi / 2 - w_stop):
                state = 3
                v_ref = 0
                w_ref = 0
            elif state == 3 and (w <= 0):
                state = 4
                v_ref = v_max
                w_ref = 0

            elif state == 4 and (y >= box_width - v_stop):
                state = 5
                v_ref = 0
                w_ref = 0
            elif state == 5 and (v <= 0):
                state = 6
                v_ref = 0
                w_ref = -w_max
            elif state == 6 and (theta <= 0 + w_stop):
                state = 7
                v_ref = 0
                w_ref = 0
            elif state == 7 and (w >= 0):
                state = 8
                v_ref = v_max
                w_ref = 0

            elif state == 8 and (x >= 2 * cycle * box_width - v_stop):
                state = 9
                v_ref = 0
                w_ref = 0
            elif state == 9 and (v <= 0):
                state = 10
                v_ref = 0
                w_ref = -w_max
            elif state == 10 and (theta <= -np.pi / 2 + w_stop):
                state = 11
                v_ref = 0
                w_ref = 0
            elif state == 11 and (w >= 0):
                state = 12
                v_ref = v_max
                w_ref = 0

            elif state == 12 and (y <= 0 + v_stop):
                state = 13
                v_ref = 0
                w_ref = 0
            elif state == 13 and (v <= 0):
                state = 14
                v_ref = 0
                w_ref = w_max
            elif state == 14 and (theta >= 0 - w_stop):
                state = 15
                v_ref = 0
                w_ref = 0
            elif state == 15 and (w <= 0):
                state = 0
                v_ref = v_max
                w_ref = 0
                cycle += 1

        x = x + v * dt * np.cos(theta + w * dt / 2)
        y = y + v * dt * np.sin(theta + w * dt / 2)
        theta = theta + w * dt

    data.append([t, x, y, theta, v, w])
    if cycle >= box_num: break

comment = '# Time [sec], X [m], Y [m], Theta [rad], LinVel [m/s], AngVel [rad/s]'
np.savetxt(save_file, data, fmt='%.6f', delimiter=', ', header=comment)

data_array = np.array(data)
plt.plot(data_array[:,1], data_array[:,2])
plt.gca().set_aspect('equal')
