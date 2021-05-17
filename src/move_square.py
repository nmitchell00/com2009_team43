#!/usr/bin/env python
# a template for the move_square exercise

import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

class move_square:
    
    def callback_function(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can 
        # access them elsewhere within our move_square class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        # If this is the first time that this callback_function has run, then 
        # obtain the "initial robot pose"
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the robot starting position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() to switch between turning and moving forwards?...
        self.turn = False

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node('movesquare_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        # define the robot pose variables and set them all to zero to start with:
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.counter = 0
        
        # define a Twist instance, which can be used to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the movesquare node has been initialised...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        # publish an empty twist message to stop the robot (by default all 
        # velocities within this will be zero):
        self.pub.publish(Twist())
    
    def print_stuff(self, a_message):
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        print(a_message)
        # you could use this to print the current velocity command:
        print("current velocity: lin.x = {:.1f}, ang.z = {:.1f}".format(self.vel.linear.x, self.vel.angular.z))
        # you could also print the current odometry to the terminal here, if you wanted to:
        print("current odometry: x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.x, self.y, self.theta_z))
        print("original odometry: x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.x0, self.y0, self.theta_z0))
        print("difference: x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(abs(self.x-self.x0), abs(self.y-self.y0), abs(self.theta_z-self.theta_z0)))
        print("counter: {:.3f}".format(self.counter))
    def main_loop(self):
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your 
            # robot. Add code here to make your robot move in a square of
            # dimensions 0.5x0.5m...
            if self.counter < 8:
                #self.vel.angular.z = 1
                if self.turn == False:
                    if (abs(self.x-self.x0) > 0.505) or (abs(self.y-self.y0) > 0.51):
                    #elif (self.x-self.x0) > 0.51:
                        self.vel.linear.x = -0.05
                    elif (abs(self.x-self.x0) > 0.50) or (abs(self.y-self.y0) > 0.5):
                        self.vel.linear.x = 0
                        self.turn = True
                    else:
                        self.vel.linear.x = 0.1
                        
                
                if self.turn == True:
                    if (abs(self.theta_z-self.theta_z0) > 1.58 and abs(self.theta_z-self.theta_z0) < 4.7):
                        self.vel.angular.z = 0.1
                    #elif (abs(self.theta_z-self.theta_z0) > 1.58
                    elif (abs(self.theta_z-self.theta_z0) > pi/2):
                        self.vel.angular.z = 0
                        self.x0 = self.x
                        self.y0 = self.y
                        self.theta_z0 = self.theta_z
                        self.turn = False
                        self.counter = self.counter+1
                    else:
                        self.vel.angular.z = -0.3
                    
            # my difference: x = 0.001, y = 0.000, theta_z = 0.012
            
            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # call a function which prints some information to the terminal:
            self.print_stuff("this is a message that has been passed to the print_stuff() method")
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == '__main__':
    movesquare_instance = move_square()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass