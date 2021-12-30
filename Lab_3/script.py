from time import sleep
#from drive import RosAriaDriver
import math
from math import fabs, sin, cos
import json
import matplotlib.pyplot as plt
import numpy as np

	
def pol2cart(rho, phi):
	x = rho*np.cos(phi)
	y = rho*np.sin(phi)
	return(x,y)

def lines2Points(lines):	# Nie pokazuje pierwszej lini !!
	X = []
	Y = []
	for l in range(len(lines)-1):
		x1, y1, a1, b1, phi1 = lines[l]		# p1
		x2, y2, a2, b2, phi2 = lines[l+1]	# p2
		A = [[a1,-1],
			 [a2,-1]]
		B =	[[-b1],
			 [-b2]]
		C = np.linalg.inv(A).dot(B)
		X.append(C[0])
		Y.append(C[1])
	return [X,Y]
		
def points2Lines(X, Y, buf):
	Lines = []
	n = len(X)
	p = 0
	while True:
		x = X[p:p+buf]
		y = Y[p:p+buf]
		#print(x)
		#print("X: " + str(x) + " Y: " + str(y))
		model = np.polyfit(np.array(x), np.array(y) , 1)
		#print(model)
		#ffit = np.poly1d(model)
		a = model[0]
		#print("a = " + str(a))
		b = model[1]
		phi = np.rad2deg(np.arctan(a))
		Lines.append([x,y,a,b,phi])

		p = p+buf-1
		if n - p < buf-1:
			if n - p > 0:
				# zostaly punkty jeszcze
				buf = n-p+1
			else:
				return Lines

def sonar2Points(data):
	X = []
	Y = []
	for p in range(512):
		phi = 360/512 * p/2
		if (math.isinf(data[p]) != True) and (math.isnan(data[p]) != True):
			x, y =pol2cart(data[p], np.deg2rad(phi))
			#points.append([x,y])
			X.append(x)
			Y.append(y)
			#print(data[p])
	return X,Y

def filterLines(lines, sens):
	new_lines = []
	old_count = len(lines)
	while True:
		x1, y1, a1, b1, phi1 = lines.pop(0)		# p1
		x2, y2, a2, b2, phi2 = lines.pop(0) 	# p2
		x = x2.copy()
		y = y2.copy()
		# Jesli roznica miedzy dwoma liniami jest ponizej sens to scal je
		#print("phi1 = " + str(phi1) + ", phi2 = " + str(phi2))
		if math.fabs(phi1 - phi2) <= sens:
			#print("scalenie")
			# transfer p1 do p2
			while len(x1) > 0:
				exist = False
				for i in range(len(x2)):
					if (x1[0] == x2[i]) and (y1[0] == y2[i]):
						# w p2 jest punkt z p1, nie dodajemy go do p2
						exist = True
						#print("exists")
						continue
				if exist == False:
					x.append(x1[0])
					y.append(y1[0])
				x1.pop(0)
				y1.pop(0)
			model = np.polyfit(np.array(x), np.array(y) , 1)
			a = model[0]
			b = model[1]
			phi = np.rad2deg(np.arctan(a))
			# Zapisanie scalonych lini
			new_lines.append([x,y,a,b,phi])
		else:
			new_lines.append([x1,y1,a1,b1,phi1])
			new_lines.append([x2,y2,a2,b2,phi2])
		if len(lines) <= 2:
			while len(lines) > 0:
				new_lines.append(lines.pop(0))
			if old_count > len(new_lines):
				new_lines = filterLines(new_lines, sens)
			return new_lines
		#print("x2: " + str(x))
		#input("")
	
###################################################################################
# BEGIN
###################################################################################
poly_points = 4		# GOLDEN VALUES = 4 AND 17
filtr_deg 	= 45/2

fig2 = plt.figure()
ax2 = fig2.gca()

json_data = open('box.json')
data = json.load(json_data)

x = np.arange(0,512)
theta = (np.pi/512 )*x  # theta - scan angles in [rad]
X,Y = sonar2Points(data)
print("Ilosc wczytanych punktów: " + str(len(X)))

lines = points2Lines(X, Y, poly_points) 
print("Ilosc lini surowych: " + str(len(lines)))

pol_X, pol_Y = lines2Points(lines)
lines = filterLines(lines, filtr_deg)
print("Ilosc lini po filtracji [deg=" + str(filtr_deg) + "] : " + str(len(lines)))

new_X, new_Y = lines2Points(lines)

line, = ax2.plot(X,Y,'.')
line, = ax2.plot(pol_X,pol_Y)
line, = ax2.plot(new_X,new_Y)


plt.show()
	
