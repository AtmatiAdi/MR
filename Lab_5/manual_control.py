import signal
from xbox360controller import Xbox360Controller
from geometry_msgs.msg import Twist
import rospy

pub = None

def on_trigger_pressed(button):
    print('Button {0} was pressed'.format(button.name))

def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    twist = Twist()
    if abs(axis.x) > 0.01:
        twist.angular.z = -axis.x 
    else:
        twist.angular.z = 0
    if abs(axis.y) > 0.01:
        twist.linear.x = -axis.y        
    else:
        twist.linear.x = 0
    pub.publish(twist)

if __name__=="__main__":
    nr = 6
    rospy.init_node('XboxController', anonymous=True)    
    pub = rospy.Publisher('/PIONIER'+str(nr)+'/RosAria/cmd_vel', Twist)
    try:
        with Xbox360Controller(0, axis_threshold=0.2) as controller:

            # Left and right axis move event
            controller.button_trigger_r.when_pressed = on_trigger_pressed
            controller.axis_l.when_moved = on_axis_moved
            controller.axis_r.when_moved = on_axis_moved

            signal.pause()
    except KeyboardInterrupt:
        pass

    rospy.spin()

