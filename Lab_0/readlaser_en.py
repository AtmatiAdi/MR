#!/usr/bin/env python
#  RosAriaDriver class documentation 
# http://panamint.ict.pwr.wroc.pl/~dbaransk/rosaria_drive/dox/html/index.html
from time import sleep
from drive import RosAriaDriver

# Enter robot number instead of X 
robot=RosAriaDriver('/PIONIER5')
# Read scanner 
scan=robot.ReadLaser()
# Print all readings 
print(scan)
# Number of readings 
print(len(scan))
# Print 6th element 
print(scan[5])
# Print 6 consecutive scans 
print(scan[20:25])

