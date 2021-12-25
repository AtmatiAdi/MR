#!/usr/bin/env python
from time import sleep
from drive import RosAriaDriver

from math import sin, cos

#Replace X with robot number
robot=RosAriaDriver('/PIONIER5')
skan=robot.ReadLaser()

### read and write in json format
#import json
## print to stdout
#print(json.dumps(skan))
## read data from file
#json_data = open('skan.json')
#data = json.load(json_data)

import matplotlib.pyplot as plt
import numpy as np

x = np.arange(0,512)
theta = (np.pi/512 )*x  # angle in rad 

fig1 = plt.figure()
ax1 = fig1.add_axes([0.1,0.1,0.8,0.8],polar=True)
line, = ax1.plot(theta,skan,lw=2.5)
ax1.set_ylim(0,2)  # distance range
plt.show()

