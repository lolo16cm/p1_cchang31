#!/usr/bin/env python3
import rospy
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from std_srvs.srv import Empty
from math import pow, atan2, sqrt

# Global variables to store positions
hunter_pose = Pose()
runner_pose = Pose()

def reset():
    """Resets the environment"""
    rospy.wait_for_service('/reset')
    try:
        reset_serv = rospy.ServiceProxy('/reset', Empty)
        reset_serv()
    except rospy.ServiceException as e:
        rospy.logerr(f"Reset failed: {e}")

def update_hunter_pose(data):
    """Subscriber callback function to update hunter pose."""
    global hunter_pose
    hunter_pose = data

def update_runner_pose(data):
    """Subscriber callback function to update runner pose."""
    global runner_pose
    runner_pose = data

def euclidean_distance(p1, p2):
    """Calculates distance between hunter and runner."""
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))

def steering_angle(p1, p2):
    """Calculates target heading."""
    return atan2(p2.y - p1.y, p2.x - p1.x)

def spawn_turtle(x, y, theta, name):
    """ Spawn a turtle."""
    rospy.wait_for_service('/spawn')
    try:
        spawn_serv = rospy.ServiceProxy('/spawn', Spawn)
        spawn_serv(x, y, theta, name)
        rospy.loginfo(f"Spawned {name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn failed: {e}")

def kill_turtle(name):
    """Kill the turtle."""
    rospy.wait_for_service('/kill')
    try:
        kill_serv = rospy.ServiceProxy('/kill', Kill)
        kill_serv(name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Kill failed: {e}")

def enforce_bounds(pose, name):
    """The turtle repositions to the center when it hits the walls."""
    if pose.x > 10.7 or pose.x < 0.3 or pose.y > 10.7 or pose.y < 0.3:
        service_name = f'/{name}/teleport_absolute' #service
        rospy.wait_for_service(service_name)
        try:
            teleport = rospy.ServiceProxy(service_name, TeleportAbsolute)
            # Push back toward center
            new_x = 5.5 if (pose.x > 10.7 or pose.x < 0.3) else pose.x
            new_y = 5.5 if (pose.y > 10.7 or pose.y < 0.3) else pose.y
            teleport(new_x, new_y, pose.theta)
        except rospy.ServiceException:
            pass

if __name__ == '__main__':
    try:
        rospy.init_node('catch_game', anonymous=True)
        reset()
        
        # Publishers 
        hunter_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        runner_pub = rospy.Publisher('/runner/cmd_vel', Twist, queue_size=10)
        
        # Subscribers: listen to the move and update the position.
        rospy.Subscriber('/turtle1/pose', Pose, update_hunter_pose)
        rospy.Subscriber('/runner/pose', Pose, update_runner_pose)
        
        rate = rospy.Rate(15)
        
        while not rospy.is_shutdown():
            # 1. Spawn runner at random position 
            # The turtlesim window is a square grid that typically ranges from 0.0 to 11.0
            spawn_turtle(random.uniform(1.0, 10.0), random.uniform(1.0, 10.0), 0, 'runner')
            rospy.sleep(1)
            last_change = rospy.get_time()
            current_rotation = random.uniform(-1.0, 1.0) # Initial range [-1,1] 
            
            # Chase Loop
            while not rospy.is_shutdown():
                dist = euclidean_distance(hunter_pose, runner_pose)
                
                # Catch condition: kill and respawn if distance < 1.0 
                if dist < 1.0:
                    kill_turtle('runner')
                    break
                
                # Repositions to the center when it hits the walls
                enforce_bounds(hunter_pose, 'turtle1')
                enforce_bounds(runner_pose, 'runner')

                # Runner Behavior: Linear 1.0 , Rotation changes every 2s 
                if rospy.get_time() - last_change >= 2.0:
                    current_rotation = random.uniform(-1.0, 1.0) 
                    last_change = rospy.get_time()
                
                runner_vel = Twist()
                runner_vel.linear.x = 1.0 # Constant linear velocity 
                runner_vel.angular.z = current_rotation
                runner_pub.publish(runner_vel)
                
                # Hunter Behavior: Max linear velocity 1.0 
                hunter_vel = Twist()
                hunter_vel.linear.x = 1.0 
                
                # Calculate steering angle
                angle_to_target = steering_angle(hunter_pose, runner_pose)
                angle_error = angle_to_target - hunter_pose.theta
                # returns an angle between -pi and pi:-340 degree is the same as +20
                normalized_error = atan2(math.sin(angle_error), math.cos(angle_error)) 
                
                hunter_vel.angular.z = 6.0 * normalized_error 
                hunter_pub.publish(hunter_vel)
                
                rate.sleep()
                
    except rospy.ROSInterruptException:
        pass

    