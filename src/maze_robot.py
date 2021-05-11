#! /usr/bin/python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import rospy
import actionlib
import numpy as np

from move_tb3 import MoveTB3
from math import degrees
from math import radians

class MazeRobot(object):

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('odom_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.object_distance = 1
        self.distance_right = 1
        self.distance_left = 1

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("publisher node is active...")


    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)
        print("stopping publisher node at: {}".format(rospy.get_time()))

    def main_loop(self):
        vel_cmd = Twist()
        while not self.ctrl_c:
            if self.object_distance > 0.4:
                vel_cmd.linear.x = 0.25
                vel_cmd.angular.z = 0.0
                self.pub.publish(vel_cmd)
            elif self.distance_left > 0.4:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.55
                self.pub.publish(vel_cmd)
            else:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = -0.55
                self.pub.publish(vel_cmd)

        #self.turn_left(3)
        #self.turn_right(3)
        #self.move_forward(3)
        #self.move_backwards(3)
        #print(object_distance)
        #self.robot_controller = MoveTB3()

    def callback_lidar(self, lidar_data):
        left_arc = lidar_data.ranges[0:35]
        right_arc = lidar_data.ranges[-35:]
        front_arc = np.array(left_arc + right_arc)
        # find the miniumum object distance within the frontal laserscan arc:
        self.object_distance = front_arc.min()

        left_arc = lidar_data.ranges[60:70]
        right_arc = lidar_data.ranges[70:80]
        total_arc1 = np.array(left_arc + right_arc)
        self.distance_left = total_arc1.min()

        left_arc = lidar_data.ranges[290:310]
        right_arc = lidar_data.ranges[310:330]
        total_arc2 = np.array(left_arc + right_arc)
        self.distance_right = total_arc2.min()


    def turn_left(self, time):
        i = 0
        for i in range(time):
            vel_cmd = Twist()
            vel_cmd.angular.z = 0.5
            self.pub.publish(vel_cmd)

            rospy.sleep(1)
            i=+1

        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)

    def turn_right(self, time):
        i = 0
        for i in range(time):
            vel_cmd = Twist()
            vel_cmd.angular.z = -0.5

            self.pub.publish(vel_cmd)

            rospy.sleep(1)
            i=+1

        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)

    def move_forward(self, time):
        i = 0
        for i in range(time):
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.3

            self.pub.publish(vel_cmd)

            rospy.sleep(1)
            i=+1

        vel_cmd.linear.x = 0.0
        self.pub.publish(vel_cmd)

    def move_backwards(self, time):
        i = 0
        for i in range(time):
            vel_cmd = Twist()
            vel_cmd.linear.x = -0.05
            vel_cmd.angular.z = 0.0

            self.pub.publish(vel_cmd)

            rospy.sleep(1)
            i=+1

        vel_cmd.linear.x = 0.0
        self.pub.publish(vel_cmd)



if __name__ == '__main__':
    publisher_instance = MazeRobot()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
