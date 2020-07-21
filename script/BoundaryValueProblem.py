# coding=utf-8
import math
import numpy as np
import matplotlib.pyplot as plt


class Node:

    def __init__(self, x, y, deg, v, a):
        self.x = x
        self.y = y
        self.yaw = np.deg2rad(deg)
        self.v = v
        self.a = a


class SolveQuinticPolynomial:

    def __init__(self, x0, v0, a0, xf, vf, af, T):
        self.a0 = x0
        self.a1 = v0
        self.a2 = a0 / 2.
        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xf - self.a0 - self.a1 * T - self.a2 * T ** 2,
                      vf - self.a1 - 2 * self.a2 * T,
                      af - 2 * self.a2])
        x = np.linalg.solve(A, b)
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        return self.a0 + self.a1 * t + self.a2 * t ** 2 + self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

    def calc_first_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

    def calc_second_derivative(self, t):
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

    def calc_third_derivative(self, t):
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2


def quinticPolynomials(start, goal, dt):
    T = 20.

    s_x = start.x
    s_vx = start.v * math.cos(start.yaw)
    s_ax = start.a * math.cos(start.yaw)

    g_x = goal.x
    g_vx = goal.v * math.cos(goal.yaw)
    g_ax = goal.a * math.cos(goal.yaw)

    xqp = SolveQuinticPolynomial(s_x, s_vx, s_ax, g_x, g_vx, g_ax, T)

    s_y = start.y
    s_vy = start.v * math.sin(start.yaw)
    s_ay = start.a * math.sin(start.yaw)

    g_y = goal.y
    g_vy = goal.v * math.sin(goal.yaw)
    g_ay = goal.a * math.sin(goal.yaw)

    yqp = SolveQuinticPolynomial(s_y, s_vy, s_ay, g_y, g_vy, g_ay, T)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    for t in np.arange(0.0, T + dt, dt):
        time.append(t)
        rx.append(xqp.calc_point(t))
        ry.append(yqp.calc_point(t))

        vx = xqp.calc_first_derivative(t)
        vy = yqp.calc_first_derivative(t)
        v = np.hypot(vx, vy)
        yaw = math.atan2(vy, vx)
        rv.append(v)
        ryaw.append(yaw)

        ax = xqp.calc_second_derivative(t)
        ay = yqp.calc_second_derivative(t)
        a = np.hypot(ax, ay)
        if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
            a *= -1
        ra.append(a)

        jx = xqp.calc_third_derivative(t)
        jy = yqp.calc_third_derivative(t)
        j = np.hypot(jx, jy)
        if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
            j *= -1
        rj.append(j)

    for i, _ in enumerate(rx):
        plt.cla()
        plt.grid(True)
        plt.axis("equal")
        plot_arrow(start.x, start.y, start.yaw)
        plot_arrow(goal.x, goal.y, goal.yaw)
        plot_arrow(rx[i], ry[i], ryaw[i])
        plt.title("Time[s]:" + str(time[i])[0:4] +
                  " v[m/s]:" + str(rv[i])[0:4] +
                  " a[m/ss]:" + str(ra[i])[0:4] +
                  " jerk[m/sss]:" + str(rj[i])[0:4],
                  )
        plt.pause(0.001)

    return time, rx, ry, ryaw, rv, ra, rj


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)
