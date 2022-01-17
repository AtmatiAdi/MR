from time import sleep
import math
from math import fabs, sin, cos
import json
import matplotlib.pyplot as plt
import numpy as np
import cv2
import random


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return (x, y)


def lines2Points(lines):
    X = []
    Y = []
    for l in range(len(lines) - 1):
        x1, y1, a1, b1, phi1 = lines[l]  # p1
        x2, y2, a2, b2, phi2 = lines[l + 1]  # p2
        A = [[a1, -1],
             [a2, -1]]
        B = [[-b1],
             [-b2]]
        C = np.linalg.inv(A).dot(B)
        X.append(C[0])
        Y.append(C[1])
    return [X, Y]


def points2Lines(X, Y, buf):
    Lines = []
    n = len(X)
    for p in range(n - buf):
        x = X[p:p + buf]
        y = Y[p:p + buf]
        print("X: " + str(x) + " Y: " + str(y))
        model = np.polyfit(np.array(x), np.array(y), 1)
        # print(model)
        # ffit = np.poly1d(model)
        a = model[0]
        b = model[1]
        phi = np.rad2deg(np.arctan(a))
        Lines.append([x, y, a, b, phi])
    return Lines


def sonar2Points(data):
    X = []
    Y = []
    for p in range(512):
        phi = 360 / 512 * p / 2
        if (math.isinf(data[p]) != True) and (math.isnan(data[p]) != True):
            x, y = pol2cart(data[p], np.deg2rad(phi))
            # points.append([x,y])
            X.append(x)
            Y.append(y)
        # print(data[p])
    return X, Y


def dist_point_to_line(x1, y1, a, b, c):
    d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b))
    return d


# READ JSON FILE
json_data = open('box.json')
data = json.load(json_data)

# GENERATE POINTS IN CARTESIANPLANE
x = np.arange(0, 512)
theta = (np.pi/512)*x  # theta - scan angles in [rad]
X, Y = sonar2Points(data)

# PLOT
fig = plt.figure()
ax = fig.gca()
plot_line = ax.plot(X, Y, '.')
plt.xlim([-5, 5])
plt.ylim([-5, 5])
plt.savefig('tmp_map.png', dpi=200)

iterations = 10000

for j in range(1, iterations):
    X_random = []
    Y_random = []
    for i in range(0, 15):
        random_index = random.randint(0, len(X)-1)
        X_random.append(X[random_index])
        Y_random.append(Y[random_index])

    tmp_line = np.polyfit(np.array(X_random), np.array(Y_random), 1)

    threshold = 0.02
    fitted_pts_count = 0

    for i in range(0, len(X)-1):
        if dist_point_to_line(X[i], Y[i], tmp_line[0], -1, tmp_line[1]) < threshold:
            fitted_pts_count = fitted_pts_count + 1

    if fitted_pts_count > 30:
        x0 = -2.5
        y0 = tmp_line[0] * x0 + tmp_line[1]
        x1 = 1
        y1 = tmp_line[0] * x1 + tmp_line[1]
        plot_line = ax.plot([x0, x1], [y0, y1])

        pts_to_delete = []
        for i in range(0, len(X) - 1):
            if dist_point_to_line(X[i], Y[i], tmp_line[0], -1, tmp_line[1]) < threshold:
                pts_to_delete.append(i)

        for i in range(len(pts_to_delete) - 1, 0, -1):
            X.pop(pts_to_delete[i])
            Y.pop(pts_to_delete[i])


plot_line = ax.plot(X, Y, 'r.')
plt.show()
