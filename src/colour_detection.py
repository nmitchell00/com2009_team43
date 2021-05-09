#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

from geometry_msgs.msg import Twist

class colour_detection(object):

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('odom_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        #rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
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
        self.m00_min = 10000

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
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
        
        while self.pillar_detection:

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
        vel_cmd = Twist()

        while not self.ctrl_c:
            while self.pillar_detection == False and not self.ctrl_c:


                print("Happy Noises")

                i = 0
                while i < 25:
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                    self.robot_controller.publish()
                    self.rate.sleep()
                    i += 1
                    print(i)

                i = 0
                while i < 25:
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.rate.sleep()
                    i += 1
                    print(i)

                i = 0
                while i < 25:
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_reverse)
                    self.robot_controller.publish()
                    self.rate.sleep()
                    i += 1
                    print(i)

                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
                self.rate.sleep()


                print("SEARCH INITIATED: The target colour is green")

                #self.search_colour = "green"
                #self.pillar_detection = True

                self.ctrl_c = True
                
                # turn 90 degrees
                # detect colour, declare correct colour in self.search_colour
                # turn back 90 degrees
                # move forward 0.75m (doesn't have to be exact)
                # turn 90 degrees anti-clockwise
                # declare self.pillar_detection = True
                
            while self.pillar_detection == True and not self.ctrl_c:
                if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'

                    
                if self.move_rate == 'fast':
                    print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop':
                    print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.ctrl_c = True
                else:
                    print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                
                self.robot_controller.publish()
                self.rate.sleep()
            
if __name__ == '__main__':
    search_ob = colour_detection()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
