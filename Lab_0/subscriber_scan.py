#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
from sensor_msgs.msg import LaserScan
import tf

def callback_scan(msg):
    ## extract ranges from message
    scan=list(msg.ranges)
    print "  Scan min: %f  front: %f" % ( min(scan),scan[256])
    print 

    
def listener():
    rospy.init_node('listener', anonymous=True)

    # Subscribe topics and bind with callback functions
    rospy.Subscriber("/PIONIER"+nr+"/scan", LaserScan, callback_scan)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] <robot_number>\n')
        sys.exit(1)

    nr=sys.argv[1]
    listener()
