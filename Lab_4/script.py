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
		X.append(C[0][0])
		Y.append(C[1][0])
	return [X,Y]
		
def points2Lines(X, Y, buf):
	Lines = []
	n = len(X)
	p = 0
	while True:
		x = X[p:p+buf]
		y = Y[p:p+buf]
		#print(x)
		print("X: " + str(x) + " Y: " + str(y))
		model = np.polyfit(np.array(x), np.array(y) , 1)
		print(model)
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
    Alph = []
    Dist = []
    for p in range(512):
        phi = 360 / 512 * p / 2
        if (math.isinf(data[p]) != True) and (math.isnan(data[p]) != True):
            x, y = pol2cart(data[p], np.deg2rad(phi))
            # points.append([x,y])
            X.append(x)
            Y.append(y)
            Alph.append(phi)
            Dist.append(data[p])
        # print(data[p])
    return X, Y, Alph, Dist

def filterLines(lines, step, max):
	filter_val = 0

	while filter_val < max:
		filter_val += step
		if filter_val > max: filter_val = max
		while True:
			# Filter lines unless nothing can be filterred using these filter_val
			old_count = len(lines)
			lines = filterLinesCore(lines, filter_val)
			#print("reduced: " + str(old_count - len(lines)) + " deg: " + str(filter_val))
			if old_count == len(lines): break
	return lines

def filterLinesCore(lines, sens):
	new_lines = []
	old_count = len(lines)
	while True:
		x1, y1, a1, b1, phi1 = lines.pop(0)		# p1
		x2, y2, a2, b2, phi2 = lines.pop(0) 	# p2
		x = x2.copy()
		y = y2.copy()
		# Jesli roznica miedzy dwoma liniami jest ponizej sens to scal je
		#print("phi1 = " + str(phi1) + ", phi2 = " + str(phi2))
		if (math.fabs(phi1 - phi2) <= sens) or ( (90 - math.fabs(phi1)) + (90 - math.fabs(phi2)) <= sens): # 
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
			return new_lines
		#print("x2: " + str(x))
		#input("")

def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        return (x3, y3, x4, y4)
###################################################################################
# BEGIN
###################################################################################
poly_points 	= 4	
poly_points_2 	= 3
filtr_step 		= 1
filtr_deg 		= 10
filtr_deg_2 	= 15

fig2 = plt.figure()
ax2 = fig2.gca()

# READ JSON FILE
json_data = open('line.json')
raw_data = json.load(json_data)

print("Keys of iteration 0:")
print(raw_data[0].keys())

print("Angles to markers in iteration 0:")

# GENERATE POINTS IN CARTESIANPLANE
data = raw_data[2]['scan']
x = np.arange(0,512)
theta = (np.pi/512 )*x  # theta - scan angles in [rad]
X,Y,Alph,Dist = sonar2Points(data)

print("Ilosc wczytanych punktów: " + str(len(X)))

# GENERATE LINES USING POLYFIT
lines = points2Lines(X, Y, poly_points) 
print("Ilosc lini surowych: " + str(len(lines)))
pol_X, pol_Y = lines2Points(lines)

# LINES FILTRATION
lines = filterLines(lines, filtr_step, filtr_deg)
print("Ilosc lini po filtracji [deg=" + str(filtr_deg) + "] : " + str(len(lines)))
new_X, new_Y = lines2Points(lines)

# SECOND GENERATION LINES USING POLYFIT
lines = points2Lines(new_X, new_Y, poly_points_2) 
new_pol_X, new_pol_Y = lines2Points(lines)

# SECOND LINES FILTRATION
lines = filterLines(lines, filtr_step, filtr_deg_2)
new_new_X, new_new_Y = lines2Points(lines)
print("Ilosc lini po filtracji [deg=" + str(filtr_deg_2) + "] : " + str(len(lines)))

line, = ax2.plot(X,Y,'.')
#line, = ax2.plot(pol_X,pol_Y)
#line, = ax2.plot(new_X,new_Y)
#line, = ax2.plot(new_pol_X,new_pol_Y)
line, = ax2.plot(new_new_X,new_new_Y)
line, = ax2.plot(new_new_X,new_new_Y, '.')
#plt.xlim([-5, 5])
#plt.ylim([-5, 5])

one = 73
two = 332


# intersection circles
x0, y0 = X[one], Y[one]
r0 = Dist[one]
x1, y1 = X[two], Y[two]
r1 = Dist[two]

plt.plot(x0,y0, '.', color='r')
plt.plot(x1,y1, '.', color='r')

circle1 = plt.Circle((x0, y0), r0, color='b', fill=False)
circle2 = plt.Circle((x1, y1), r1, color='b', fill=False)

#fig, ax = plt.subplots() 
#ax2.set_xlim((-10, 10))
#ax2.set_ylim((-10, 10))
ax2.add_artist(circle1)
ax2.add_artist(circle2)

intersections = get_intersections(x0, y0, r0, x1, y1, r1)
if intersections is not None:
    i_x3, i_y3, i_x4, i_y4 = intersections 
    plt.plot([i_x3, i_x4], [i_y3, i_y4], '.', color='b')
    

plt.gca().set_aspect('equal', adjustable='box')

plt.show()
	
