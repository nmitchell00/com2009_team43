#! /usr/bin/python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import rospy
import actionlib

from move_tb3 import MoveTB3
from math import degrees
from math import radians

class ExploringRobot(object):

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('odom_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("publisher node is active...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))

    def main_loop(self):
        self.turn_left()
        self.turn_right()

        #self.robot_controller = MoveTB3()

    def turn_left(self):
        i = 0
        for i in range(3):
            vel_cmd = Twist()
            vel_cmd.angular.z = 0.8

            self.pub.publish(vel_cmd)

            rospy.sleep(3)
            print("looped " + str(i))
            i=+1

        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)

    def turn_right(self):
        i = 0
        for i in range(3):
            vel_cmd = Twist()
            vel_cmd.angular.z = -0.8

            self.pub.publish(vel_cmd)

            rospy.sleep(3)
            print("looped " + str(i))
            i=+1

        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)

if __name__ == '__main__':
    publisher_instance = ExploringRobot()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
