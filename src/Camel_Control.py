#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,Pose
from rospy.client import get_param, has_param
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
import time
from math import radians, sqrt, pi , degrees 
import PyKDL
class CamelControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/camel_amr_1000_001/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/camel_amr_1000_001/odometry', Odometry, self.odom_callback)
        self.cmd = Twist()
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.angular_tolerance = radians(1)
        self.position = Odometry().pose.pose.position
        self.yaw = 0
        self.velocity = Odometry().twist.twist
        #self.linear_tolerance = rospy.get_param('linear_tolerance')
        #self.pitch_tolerance = rospy.get_param('pitch_tolerance')
        #self.tolerance_x = rospy.get_param('tolerance_x_for_centering')
        rospy.on_shutdown(self.shutdownhook)
        self.ctrl_c = False
        #self.parking_stop_distance = rospy.get_param('parking_stop_distance')
        #self.check_1_point_stop_distance = rospy.get_param('check_1_point_stop_distance')
        #self.obstacle_safety_distance = rospy.get_param('safety_distance')
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
        
            if connections > 0 :
                self.vel_publisher.publish(self.cmd)
                # rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):       
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

    def odom_callback(self, msg : Odometry):
        time.sleep(0.1)
        self.position = msg.pose.pose.position
        #self.position_x = msg.pose.pose.position.x 
        #self.position_y = msg.pose.pose.position.y
        #self.position_z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        self.velocity = msg.twist.twist 
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def get_odom(self):
        time.sleep(0.1)
        return self.position, self.yaw
    
    def get_velocity(self):
        time.sleep(0.1)
        return self.velocity.linear.x, self.velocity.angular.z

    def get_laser(self, pos):
        time.sleep(0.1)
        return self.laser_msg.ranges[pos]

    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_back_laser(self):
        time.sleep(0.4)
        return self.laser_msg.ranges[0]
    
    def obstacle_avoidance(self):
        #time.sleep(0.01)
        self.regions = {
            'full':  min(min(self.laser_msg.ranges[0:1083]), 10),
        }
        return self.regions['full']
    
    def get_laser_full(self):
        time.sleep(0.005)
        return self.laser_msg.ranges
            
    def stop_robot(self):
        # rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
    
    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):
            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s

    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)

        while (i <= time):
            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s

    def move(self, distance):
        position = Point()
        time.sleep(2)
        # Get the current position
        (position, rotation) = self.get_odom()
        (linear_speed, angular_speed) = self.get_velocity()
        # Set the movement command to a forward or backward
        if distance > 0:
            self.cmd.linear.x = 0.2
            k = 0.4
        else:
            self.cmd.linear.x = -0.2
            k = -0.4
        # Track the last point measured
        last_point_x = position.x
        last_point_y = position.y

        # Track how far we have turned
        move_distance = 0

        goal_distance = distance
        linear = self.cmd.linear.x
        # Begin the motion
        delta_distance = 0

        while abs(move_distance) < abs(goal_distance) and not rospy.is_shutdown():
            #Publish the Twist message and sleep 1 cycle
            #self.cmd.linear.x = linear * (abs(goal_distance) - abs(move_distance))
            if(abs(goal_distance) - abs(move_distance) > 0.4):

                self.cmd.linear.x = self.cmd.linear.x
            else:
                self.cmd.linear.x = k * (abs(goal_distance) - abs(move_distance))
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_distance = sqrt((position.x - last_point_x) ** 2 + (position.y - last_point_y) ** 2)

            move_distance += delta_distance
            last_point_x = position.x
            last_point_y = position.y

        self.stop_robot()

    def rotate(self, degree):

        position = Point()

        # Get the current position
        time.sleep(2)
        (position, rotation) = self.get_odom()

        # Set the movement command to a rotation
        if degree > 0:
            self.cmd.angular.z = 0.2
            k = 0.7
        else:
            self.cmd.angular.z = -0.2
            k = -0.7
        # Track the last angle measured
        last_angle = rotation

        # Track how far we have turned
        turn_angle = 0
        
        goal_angle  = radians(degree) 
        angular = self.cmd.angular.z
        # Begin the rotation
        while abs(turn_angle) + 0.052 < abs(goal_angle) and not rospy.is_shutdown():
            print("rotation: ",degrees(rotation))
            print(degrees(goal_angle) , degrees(turn_angle))
            print('diff: ', abs(goal_angle) - abs(turn_angle))
            # Publish the Twist message and sleep 1 cycle
            if(abs(goal_angle) - abs(turn_angle) > 0.2):
                self.cmd.angular.z = angular            
            else:
                self.cmd.angular.z = k * (abs(goal_angle) - abs(turn_angle))
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()
           
            # Compute the amount of rotation since the last lopp
            rotation = self.normalize_angle(rotation)
            delta_angle = self.normalize_angle(rotation - last_angle)

            turn_angle += delta_angle
            last_angle = rotation
            

        self.stop_robot()

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def quat_to_degree(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return degrees(rot.GetRPY()[1])

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == '__main__':
    #rospy.init_node('Camel_control_node', anonymous=True)
    robotcontrol_object = CamelControl()
    try:
        robotcontrol_object.get_odom()

    except rospy.ROSInterruptException:
        pass