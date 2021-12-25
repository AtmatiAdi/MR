#!/usr/bin/env python
from time import sleep
from drive import RosAriaDriver

import sys
if (len(sys.argv)!=2):
    print 'Run: ', sys.argv[0],'robot_number > filename' 
    sys.exit()


robot=RosAriaDriver('/PIONIER'+sys.argv[1])

#robot.SetSpeed(0.1,0,0.5)

skan=robot.ReadLaser()

import json
print(json.dumps(skan))

#json_data = open('skan.json')
#data = json.load(json_data)
#print data

