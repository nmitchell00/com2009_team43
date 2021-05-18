#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import numpy as np

# Import some other modules from within this package
from move_tb3 import MoveTB3

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class SearchBeacon(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.object_distance = 1
        self.distance_right = 1
        self.distance_left = 1
        self.thin_front = 1

        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.turn_vel_reverse = 0.5
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.search_colour = ""
        self.pillar_detection = False

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        self.m00 = 0
        self.m00_min = 100000

        self.startup = True

        self.Odomsub = rospy.Subscriber('odom', Odometry, self.callback_function)

        self.x = 0.0
        self.y = 0.0
        self.x0 = 0.0
        self.y0 = 0.0


    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def callback_function(self, odom_data):

        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        self.x = pos_x
        self.y = pos_y

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 250
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lowerblue = (115, 224, 100)
        upperblue = (130, 255, 255)

        lowerred = (-2,240,100)
        upperred = (5,255,255)

        lowergreen = (40,205,100)
        uppergreen = (63,255,255)

        lowercyan = (85,185,100)
        uppercyan = (92,255,255)

        lowerpurple = (140,160,100)
        upperpurple = (160,255,255)

        loweryellow = (25,210,100)
        upperyellow = (32,255,255)

        if self.search_colour == "blue":
            mask = cv2.inRange(hsv_img, lowerblue, upperblue)
        if self.search_colour == "red":
            mask = cv2.inRange(hsv_img, lowerred, upperred)
        if self.search_colour == "green":
            mask = cv2.inRange(hsv_img, lowergreen, uppergreen)
        if self.search_colour == "cyan":
            mask = cv2.inRange(hsv_img, lowercyan, uppercyan)
        if self.search_colour == "purple":
            mask = cv2.inRange(hsv_img, lowerpurple, upperpurple)
        if self.search_colour == "yellow":
            mask = cv2.inRange(hsv_img, loweryellow, upperyellow)


        #mask = cv2.inRange(hsv_img, lowercyan, uppercyan) + cv2.inRange(hsv_img, lowerred, upperred) + cv2.inRange(hsv_img, lowergreen, uppergreen) + cv2.inRange(hsv_img, lowerblue, upperblue)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:

            self.detect_colour()
            #pillar_found = False
            i = 0
            for i in range(70):
                if self.distance_right > 0.375:
                    self.robot_controller.set_move_cmd(0.0, -0.325)
                elif self.object_distance < 0.375 or self.distance_right < 0.15:
                    self.robot_controller.set_move_cmd(0.0, 0.325)
                else:
                    self.robot_controller.set_move_cmd(0.25, 0.0)
                self.robot_controller.publish()
                self.rate.sleep()
                i =+ 1

            #while pillar_found == False:
            self.explore()
            self.scan_colour()
                    #pillar_found == True

            while self.thin_front > 0.4:

                self.robot_controller.set_move_cmd(0.2, 0.0)
                self.robot_controller.publish()

            #preventing buffering errors when stopping
            i = 0
            for i in range(5):
                self.robot_controller.set_move_cmd(0.0,0.0)
                self.rate.sleep()
                i += 1
            self.ctrl_c = True

    def callback_lidar(self, lidar_data):

        left_arc = lidar_data.ranges[0:30]
        right_arc = lidar_data.ranges[-30:]
        front_arc = np.array(left_arc + right_arc)
        self.object_distance = front_arc.min()

        left_arc = lidar_data.ranges[40:50]
        right_arc = lidar_data.ranges[50:70]
        total_arc1 = np.array(left_arc + right_arc)
        self.distance_left = total_arc1.min()

        left_arc = lidar_data.ranges[290:300]
        right_arc = lidar_data.ranges[300:320]
        total_arc2 = np.array(left_arc + right_arc)
        self.distance_right = total_arc2.min()

        left_arc = lidar_data.ranges[0:10]
        right_arc = lidar_data.ranges[-10:]
        front_arc2 = np.array(left_arc + right_arc)
        self.thin_front = front_arc2.min()


    def detect_colour(self):
        i = 0
        while i < 15:
            self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            self.robot_controller.publish()
            self.rate.sleep()
            i += 1

        i = 0
        while i < 5:
            self.robot_controller.set_move_cmd(0.0, 0.0)
            self.robot_controller.publish()
            self.rate.sleep()
            i += 1

        colours = ["blue","red","cyan","purple","yellow","green"]
        i = 0
        colour_found = False
        while colour_found == False:
            self.search_colour = colours[i]
            self.robot_controller.set_move_cmd(0.0, 0.0)
            self.robot_controller.publish()
            self.rate.sleep()
            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    colour_found = True
                    print("SEARCH INITIATED: The target colour is " + self.search_colour + ".")
            else:
                i += 1


    def scan_colour(self):
        i = 0
        print("Scanning for " + self.search_colour)
        finding_pillar = False
        while finding_pillar == False:
            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-50 and self.cy <= 560+50:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'


            if self.move_rate == 'fast':
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop':
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                self.robot_controller.set_move_cmd(0.0, 0.0)
                finding_pillar = True
            else:
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

            self.robot_controller.publish()
            self.rate.sleep()
            i += 1
        #preventing buffering errors when stopping
        i = 0
        for i in range(10):
            self.robot_controller.set_move_cmd(0.0,0.0)
            self.rate.sleep()
            i += 1


    def explore(self):
        search = False
        while search == False:
            if self.distance_right > 0.375:
                self.robot_controller.set_move_cmd(0.0, -0.325)
            elif self.object_distance < 0.375 or self.distance_right < 0.15:
                self.robot_controller.set_move_cmd(0.0, 0.325)
            else:
                self.robot_controller.set_move_cmd(0.25, 0.0)
            self.robot_controller.publish()
            self.rate.sleep()
            if self.m00 > self.m00_min:
                # if the robot is far enough away from start arena
                if self.distance() > 2:
                    # blob detected
                    if self.cy >= 0 and self.cy <= 1120:
                        search = True
        #preventing buffering errors when stopping
        i = 0
        for i in range(5):
            self.robot_controller.set_move_cmd(0.0,0.0)
            self.rate.sleep()
            i += 1


    def distance(self):
        xdist = self.x - self.x0
        ydist = self.y - self.y0
        distance = sqrt((xdist*xdist) + (ydist*ydist))
        return distance

if __name__ == '__main__':
    search_ob = SearchBeacon()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
