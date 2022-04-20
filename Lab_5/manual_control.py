import signal
from xbox360controller import Xbox360Controller
from geometry_msgs.msg import Twist
import rospy

pub = None
holding_auto = False
holding_manual = False
auto_twist = Twist()

def on_trigger_pressed(button):
    global holding_manual    
    print('Button {0} was pressed'.format(button.name))
    holding_manual = True

def on_trigger_released(button):
    global holding_manual
    print('Button {0} was released'.format(button.name))    
    holding_manual = False

def on_trigger_pressed2(button):
    global holding_auto
    print('Button {0} was pressed'.format(button.name))
    holding_auto = True

def on_trigger_released2(button):
    global holding_auto    
    print('Button {0} was released'.format(button.name))        
    holding_auto = False
    
def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    if not holding_manual:
        print("manual motion locked!")
        return
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

def callback_mr_controller(msg):
    if not holding_auto:
        print("controller motion locked!")
        return
    if holding_manual:
        print("robot is controlled manually!")
        return    
    auto_twist = msg
    pub.publish(auto_twist)
    

if __name__=="__main__":
    nr = 6
    rospy.init_node('XboxController', anonymous=True)    
    pub = rospy.Publisher('/PIONIER'+str(nr)+'/RosAria/cmd_vel', Twist)
    rospy.Subscriber("safety_xbox_control", Twist, callback_mr_controller, queue_size=1)
    
    try:
        with Xbox360Controller(0, axis_threshold=0.2) as controller:

            # Left and right axis move event
            controller.button_trigger_r.when_pressed = on_trigger_pressed
            controller.button_trigger_r.when_released = on_trigger_released
            controller.button_trigger_l.when_pressed = on_trigger_pressed2
            controller.button_trigger_l.when_released = on_trigger_released2                        
            controller.axis_l.when_moved = on_axis_moved
            controller.axis_r.when_moved = on_axis_moved

            signal.pause()
    except KeyboardInterrupt:
        pass

    rospy.spin()

