from time import sleep
# from drive import RosAriaDriver
import math
from math import fabs, sin, cos
import json
import matplotlib.pyplot as plt
import numpy as np
import cv2


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


def lines2points(lines):
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
    lines = []
    n = len(X)
    for p in range(n - buf):
        x = X[p:p + buf]
        y = Y[p:p + buf]
        # print("X: " + str(x) + " Y: " + str(y))
        model = np.polyfit(np.array(x), np.array(y), 1)
        a = model[0]
        b = model[1]
        phi = np.rad2deg(np.arctan(a))
        lines.append([x, y, a, b, phi])
    return lines


def sonar2points(data_in):
    tmp_x_coordinates_list = []
    tmp_y_coordinates_list = []
    for p in range(512):
        phi = 360 / 512 * p / 2
        if (math.isinf(data_in[p]) != True) and (math.isnan(data_in[p]) != True):
            x_coord, y_coord = pol2cart(data_in[p], np.deg2rad(phi))
            # points.append([x,y])
            tmp_x_coordinates_list.append(x_coord)
            tmp_y_coordinates_list.append(y_coord)
        # print(data[p])
    return tmp_x_coordinates_list, tmp_y_coordinates_list


###################################################################################
# BEGIN
###################################################################################
json_data = open('box.json')
data = json.load(json_data)
x = np.arange(0, 512)
theta = (np.pi / 512) * x  # theta - scan angles in [rad]
X, Y = sonar2points(data)

fig2 = plt.figure()
ax2 = fig2.gca()
# line = ax2.plot(X, Y, '.')
plot_line = ax2.plot(X, Y, 'k.', markersize=2)

plt.axis('off')
plt.xlim([-5, 5])
plt.ylim([-5, 5])
plt.savefig('tmp_map.png', dpi=200)

img = cv2.imread('tmp_map.png')
result = img.copy()
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
kernel_size = 5
img = cv2.GaussianBlur(img,(kernel_size, kernel_size), 0)
cv2.imshow("points and lines", img)
cv2.waitKey(0)

# applying canny filter to get contours
edges = cv2.Canny(img, 50, 150)
# hough transform to get lines
lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=15, minLineLength=20, maxLineGap=10)

result1 = np.zeros(result.shape, dtype=np.uint8)
result1.fill(255)

for line in lines:
    for x1, y1, x2, y2 in line:
        cv2.line(result,  (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.line(result1, (x1, y1), (x2, y2), (0, 255, 0), 1)


cv2.imshow("points and lines", result)
cv2.imshow("only lines", result1)
cv2.waitKey(0)
