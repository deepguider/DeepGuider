import numpy as np
import matplotlib.pyplot as plt

gps_freq = 10               # [Hz]
sim_freq = 100              # [Hz]
wait_time = 0               # [sec]
traj_width = 20 * np.pi     # [m]

save_file = 'Sine(%02.0fHz,%02.0fs).pose.csv' % (gps_freq, wait_time)
dt = 1. / sim_freq

# Sine function
# y = 5 + 5 * sin(x / 2 - pi / 2)
# x = t / 2
dx = lambda t: 0.5
dy = lambda t: 5.0 / 4 * np.cos(t / 4 - np.pi / 2)
ddx = lambda t: 0
ddy = lambda t: -5.0 / 16 * np.sin(t / 4 - np.pi / 2)
get_v = lambda t: np.sqrt(dx(t) * dx(t) + dy(t) * dy(t))
get_k = lambda t: (dx(t) * ddy(t) - dy(t) * ddx(t)) / (get_v(t) ** 3)
get_w = lambda t: get_v(t) * get_k(t)

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
            v = get_v(t - wait_time)
            w = get_w(t - wait_time)

        x = x + v * dt * np.cos(theta + w * dt / 2)
        y = y + v * dt * np.sin(theta + w * dt / 2)
        theta = theta + w * dt

    data.append([t, x, y, theta, v, w])

    if x >= traj_width: break

comment = '# Time [sec], X [m], Y [m], Theta [rad], LinVel [m/s], AngVel [rad/s]'
np.savetxt(save_file, data, fmt='%.6f', delimiter=', ', header=comment)

data_array = np.array(data)
plt.plot(data_array[:,1], data_array[:,2])
plt.gca().set_aspect('equal')
