# coding=utf-8
import numpy as np
import matplotlib.pyplot as plt
from BoundaryValueProblem import Node
from BoundaryValueProblem import quinticPolynomials

def main():
    start = Node(10., 10., 0., 1., 0.1)
    goal = Node(30., 0., 0., 1., 0.1)
    dt = 0.1

    time, x, y, yaw, v, a, j = quinticPolynomials(start, goal, dt)

    plt.plot(x, y, "-r")

    plt.figure()

    plt.subplot(2, 2, 1)
    plt.plot(time, [np.rad2deg(i) for i in yaw], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Yaw[deg]")
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(time, v, "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[m/s]")
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(time, a, "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("accel[m/ss]")
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot(time, j, "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("jerk[m/sss]")
    plt.grid(True)

    plt.show()


if __name__ == '__main__':
    main()