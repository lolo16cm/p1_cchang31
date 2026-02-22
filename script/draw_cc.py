#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, Spawn, Kill
from std_srvs.srv import Empty

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('/spawn') 
    try:
        spawn_serv = rospy.ServiceProxy('/spawn', Spawn) 
        spawn_serv(x, y, theta, name) 
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn failed: {e}")


def draw_c(publisher, rate):
    vel_msg = Twist() 
    angular_val = 1.4 
    linear_val  = 2.0
    # 270 degrees = 1.5 * PI radians
    #time = distance(1.5*pi = 3/4 circle) / speed (theta val)
    duration = (1.5 * math.pi) / angular_val
    
    
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
        vel_msg.linear.x = linear_val
        vel_msg.angular.z = angular_val 
        publisher.publish(vel_msg) 
        rate.sleep() 
    
    # Stop movement 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)

def reset():
    rospy.wait_for_service('/reset') 
    try:
        reset_serv = rospy.ServiceProxy('/reset', Empty) 
        reset_serv() 
    except rospy.ServiceException as e:
        rospy.logerr(f"Reset failed: {e}")

def kill_turtle(name):
    rospy.wait_for_service('/kill')
    try:
        kill_serv = rospy.ServiceProxy('/kill', Kill)
        kill_serv(name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Kill failed: {e}")

if __name__ == '__main__':
    try:
        rospy.init_node('Draw_CC', anonymous=True)
        for i in range(2):
            reset()
            # rospy.sleep(1)
            # Kill the default turtle1
            kill_turtle('turtle1')
            # rospy.sleep(1)
            # Setup Publishers
            pub1 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
            pub2 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)
            rate = rospy.Rate(10)
            
            # Spawn and Draw First C
            spawn_turtle(3.0, 5.0, 0.75 * math.pi, 'turtle2')
            rospy.sleep(0.2)
            draw_c(pub1, rate)
            
            # Spawn and Draw Second C
            spawn_turtle(6.0, 5.0, 0.75 * math.pi, 'turtle3')
            rospy.sleep(0.2)
            draw_c(pub2, rate)
            
            rospy.loginfo("CC drawn")

    except rospy.ROSInterruptException:
        pass
