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
json_data = open('wall.json')
data = json.load(json_data)

# GENERATE POINTS IN CARTESIANPLANE
x = np.arange(0, 512)
theta = (np.pi/512)*x  # theta - scan angles in [rad]
X, Y = sonar2Points(data)

# PLOT
fig = plt.figure()
ax = fig.gca()
#plot_line = ax.plot(X, Y, '.')
#plt.xlim([-5, 5])
#plt.ylim([-5, 5])
plt.savefig('tmp_map.png', dpi=200)

neighborhood_size = 10
search_neighborhood_size = 60
pts_low_treshold = 30
pts_sense = 0.02
pts_oversense = 0.01

line_is_detected = 1

while (line_is_detected):
    best_fitted_pts_count = 0
    line_is_detected = 0
    best_random_first_index = 0
    best_line = 0
    for j in range(1, len(X)-1):
        X_random = []
        Y_random = []
        random_first_index = j
        neig_size_up = neig_size_down = neighborhood_size//2
        if (random_first_index + neig_size_up) > len(X)-1: neig_size_up = (len(X)-1) - random_first_index
        if (random_first_index - neig_size_down) < 0: neig_size_down = random_first_index
        for i in range(random_first_index - neig_size_down, random_first_index + neig_size_up):
            X_random.append(X[i])
            Y_random.append(Y[i])

        tmp_line = np.polyfit(np.array(X_random), np.array(Y_random), 1)
        fitted_pts_count = 0

        neig_size_up = neig_size_down = search_neighborhood_size//2
        if (random_first_index + neig_size_up) > len(X)-1: neig_size_up = (len(X)-1) - random_first_index
        if (random_first_index - neig_size_down) < 0: neig_size_down = random_first_index
        for i in range(random_first_index - neig_size_down, random_first_index + neig_size_up):
            if dist_point_to_line(X[i], Y[i], tmp_line[0], -1, tmp_line[1]) < pts_sense:
                fitted_pts_count = fitted_pts_count + 1

        if (fitted_pts_count > pts_low_treshold) > best_fitted_pts_count:
            best_fitted_pts_count = fitted_pts_count
            best_line = tmp_line
            line_is_detected = 1
            best_random_first_index = random_first_index

    if best_fitted_pts_count > pts_low_treshold:

        down_pts_to_delete = []
        upper_pts_to_delete = []
        # form best_random_first_index to end
        for i in range(best_random_first_index, len(X) - 1):
            if (math.dist([X[i-1], Y[i-1]], [X[i], Y[i]]) < 10*(pts_sense + pts_oversense)): 
                if dist_point_to_line(X[i], Y[i], best_line[0], -1, best_line[1]) < pts_sense + pts_oversense:
                    upper_pts_to_delete.append(i)
                else: break
        # from best_random_first_index to first
        for i in range(best_random_first_index, 0, -1):
            if (math.dist([X[i+1], Y[i+1]], [X[i], Y[i]]) < 5*(pts_sense + pts_oversense)):
                if dist_point_to_line(X[i], Y[i], best_line[0], -1, best_line[1]) < pts_sense + pts_oversense:
                    down_pts_to_delete.append(i)
                else: break

        plot_line = ax.plot([X[down_pts_to_delete[-1]], X[upper_pts_to_delete[-1]]], [Y[down_pts_to_delete[-1]], Y[upper_pts_to_delete[-1]]])
        plot_line = ax.plot([X[down_pts_to_delete[-1]], X[upper_pts_to_delete[-1]]], [Y[down_pts_to_delete[-1]], Y[upper_pts_to_delete[-1]]],'.')

        for i in range(len(upper_pts_to_delete) - 1, 0, -1):
            X.pop(upper_pts_to_delete[i])
            Y.pop(upper_pts_to_delete[i])

        for i in range(0, len(down_pts_to_delete)):
            X.pop(down_pts_to_delete[i])
            Y.pop(down_pts_to_delete[i])

#plot_line = ax.plot(X, Y, 'r.')
plt.show()
