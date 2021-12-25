import csv
import math as m
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

d = (381/2) - (45/2)
d = 0.95*d

with open('square_left.csv') as csvfile:
    reader = csv.DictReader(csvfile, delimiter=';', quotechar='|')
    enL = []
    enR = []
    velL= []
    velR= []
    T = []
    theta = []
    x = []
    y = []
    v = []
    omega = []
    enV = []
    enOmega = []
    enVelR = []
    enVelL = []
    
    i = 0
    for row in reader:        
        #print(row['posL'], row['posR'])
        if (i == 0):
          enL_0 = float(row['posL'])  
          enR_0 = float(row['posR']) 
          enVelR.append(0);
          enVelL.append(0);
          
        T.append(float(row['#time'].split(":")[2]))
        if (float(row['#time'].split(":")[1])) > 0:
          print("Time overflow")
        #print(deltaT_0)
        enL.append(float(row['posL']) - enL_0)
        enR.append(float(row['posR']) - enR_0)
        
        velL.append(float(row['velL']))
        velR.append(float(row['velR']))
        
        enVelL.append(enL[i] - enL[i-1])
        enVelR.append(enR[i] - enR[i-1])
       
        
        i = i + 1
 
    for i in range(len(velL)):
      v.append((velL[i] + velR[i])/2)
      omega.append((velR[i] - velL[i])/(2*d))
      
      enV.append((enVelL[i] + enVelR[i])/2)
      enOmega.append((enVelR[i] - enVelL[i])/(2*d))
      
      #v[i] = enV[i]
      #omega[i] = enOmega[i]
      
      if (i == 0):
        theta.append(omega[i]*T[0])
        x.append(v[i]*m.cos(theta[0])*T[0])
        y.append(v[i]*m.sin(theta[0])*T[0])
      else:
        theta.append(theta[i-1] + omega[i]*(T[i] - T[i-1]))
        x.append(x[i-1] + v[i]*m.cos(theta[i])*(T[i] - T[i-1]))
        y.append(y[i-1] + v[i]*m.sin(theta[i])*(T[i] - T[i-1]))
      
    print(d)
    fig1 = plt.figure()
    ax = fig1.gca()
    #ax.plot(T,x)
    ax.plot(x,y)
    #ax.set_xlim([-600,600])
    #ax.set_ylim([-600,600])
    ax.grid()
    plt.show()
    theta = [0]
    x = [0]
    y = [0]
      
