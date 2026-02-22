#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty # Service for resetting

def reset():
    """Resets the environment"""
    rospy.wait_for_service('/reset')
    try:
        reset_serv = rospy.ServiceProxy('/reset', Empty)
        reset_serv()
    except rospy.ServiceException as e:
        rospy.logerr(f"Reset failed: {e}")
    
def draw_c(pub, rate):
    rospy.loginfo("starting drawing C")
    msg = Twist()
    msg.linear.x = 2.0
    msg.angular.z = 1.4
    begin_time = rospy.get_time()
    while (rospy.get_time() - begin_time) < 3.3:
        pub.publish(msg)
        rate.sleep()
    pub.publish(Twist()) #stop drawing by call no value type
    rospy.sleep(0.5)

def next(x,y, theta):
        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
             teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
             teleport(x,y, theta)
        except rospy.ServiceException as e:
             rospy.logerr(f"teleport_absolute failed: {e}")
             


if __name__ == '__main__':

    try:    
        rospy.init_node("draw_name")
        rospy.loginfo("started draw_name node")
        pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) #qsize to hold the #msg
        rate = rospy.Rate(15)
        rospy.sleep(1)
        for i in range(5):
            reset()
            next(5.5, 5.5, 2.0)
            rospy.sleep(1)
            draw_c(pub, rate)
            next(6.0, 3.0, 2.5)
            rospy.sleep(1.0)
            draw_c(pub, rate)
    except rospy.ROSInterruptException:
            pass