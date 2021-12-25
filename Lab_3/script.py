from time import sleep
from drive import RosAriaDriver
import math
from math import sin, cos
import json

json_data = open('box.json')
data = json.load(json_data)

import matplotlib.pyplot as plt
import numpy as np

	
def pol2cart(rho, phi):
	x = rho*np.cos(phi)
	y = rho*np.sin(phi)
	return(x,y)

x = np.arange(0,512)
theta = (np.pi/512 )*x  # theta - scan angles in [rad]

fig1 = plt.figure()
ax1 = fig1.add_axes([0.1,0.1,0.8,0.8],polar=True) 
line, = ax1.plot(theta,data,'.',lw=1.5)
ax1.set_ylim(0,5) 

points = []
X = []
Y = []
for p in range(512):
	phi = 360/512 * p/2
	if (math.isinf(data[p]) != True) and (math.isnan(data[p]) != True):
		x, y =pol2cart(data[p], np.deg2rad(phi))
		points.append([x,y])
		X.append(x)
		Y.append(y)
		print(data[p])
n = len(points)	
lines = []
Phi = []
for p in range(n - 1):
	p1 = points[p]
	p2 = points[p+1]
	phi = np.rad2deg(np.arctan((p1[1]-p2[1])/(p1[0] - p2[0])))
	lines.append([p1,p2,phi])
	Phi.append(phi)
	
	
pol_lines = []
pol_Phi = []
buf = 5
for p in range(n-buf):
	x = X[p:p+buf]
	y = Y[p:p+buf]
	p1 = points[p]
	p2 = points[p+buf]
	#print(x)
	model = np.polyfit(np.array(x), np.array(y) , 1)
	#ffit = np.poly1d(model)
	a = 0.5
	phi = np.arctan(a)
	pol_lines.append([p1,p2,phi])
	pol_Phi.append(phi)

fig2 = plt.figure()
ax2 = fig2.gca()
line, = ax2.plot(X,Y,'.')
#plt.show()

fig3 = plt.figure()
ax3 = fig3.gca()
line, = ax3.plot(Phi)
line, = ax3.plot(pol_Phi)
plt.show()
	
