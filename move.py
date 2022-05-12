#HAJ17684295
#Kacper Hajda
#CMP3103M Assesment 1 


import rospy
import cv2
import cv_bridge
import numpy
from math import radians
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#importing the required libraries


class Controller:

    def __init__(self):
        #initalising the class

        rospy.init_node('Controller', anonymous=True)
        #initialising the node 

        self.r = rospy.Rate(10)

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # creating the necessary subscribers and publishers to ROS
        self.move_cmd = Twist()

        self.blueFlag = False
        self.redFlag = False
        self.greenFlag = False
        self.movingAway =False
        #Variables used to store if the program sees any of the colours
        self.isTurning = False

        self.initalturn = False

        self.err = None
        #used to store the error to be used to navigate towards the specified color

    def laser_callback(self, laser_data):
      

        if self.initalturn !=True:
            self.initial_turn()
    
        if laser_data.ranges[320] < 0.6:
            self.move_cmd.linear.x = -0.2
            self.cmd_vel_pub.publish(self.move_cmd)
        #does an inital spin to locate blue
        if self.movingAway !=True:
            if (laser_data.ranges[320] > 0.7 and laser_data.ranges[0] > 0.7 and laser_data.ranges[639] > 0.7) and (self.blueFlag !=False or self.greenFlag !=False) and self.redFlag !=True:
                self.move_towardsColour()
                self.isTurning = False
                #checks if the distances of the centre, left and right are further than 0.7 and sees either blue or green but not red.
            elif (laser_data.ranges[320] > 0.7 and laser_data.ranges[0] > 0.7 and laser_data.ranges[639] > 0.7) and self.redFlag != True:
                self.move_forwards()
                self.isTurning = False
                #this runs if no colour is spotted.
            elif self.redFlag == True:
                self.movingAway =True
                self.move_away()
                #executes a function to move away if red is spotted.
            else:
                self.stop_robot()
                if laser_data.ranges[639] > 0.75 and self.isTurning !=True:
                    self.turn_left()
                    #print ("death")
                elif laser_data.ranges[0] > 0.75 and self.isTurning !=True:
                    self.turn_right()
                    #print ("pain")
                #turns if either far left or far right are greater than 0.75 and it isnt already turning
                else:
                    #this is if its too close to the wall it checks which side has the higher value and rotates there
                    if max(laser_data.ranges[320:639]) > max(laser_data.ranges[0:320]) and self.isTurning !=True:
                        self.turn_left()
                    elif max(laser_data.ranges[0:320]) > max(laser_data.ranges[320:639]) and self.isTurning !=True:
                        self.turn_right()
                    else:
                        self.isTurning = False
 



    def turn_left(self):
        self.isTurning = True
        self.move_cmd.angular.z = 0.5
        for x in range(0,25):
            self.cmd_vel_pub.publish(self.move_cmd)
        #function to turn the robot left. by publishing to the twist action
        


    def turn_right(self):
        self.isTurning = True
        self.move_cmd.angular.z = -0.5
        for x in range(0,25):
            #print ("you are at road 132")
            self.cmd_vel_pub.publish(self.move_cmd)
        #function to turn the robot left. by publishing to the twist action


    def move_forwards(self):
        self.move_cmd.angular.z = 0.0
        self.move_cmd.linear.x = 0.3
        self.cmd_vel_pub.publish(self.move_cmd)
    #function to move forwards

    def initial_turn(self): # this function does an inital turn
        target_ang = radians(360)
        current_ang = 0
        #converts the angle 360 into radians and sets the inital angle to 0
        if self.initalturn != True:
            #checks if the initial turn has been done before
            t0 = rospy.Time.now().to_sec()
            #gets the time of execution
            while current_ang < target_ang:
                self.move_cmd.angular.z = radians(20)
                t1 = rospy.Time.now().to_sec()
                self.cmd_vel_pub.publish(self.move_cmd)
                current_ang = radians(10) *(t1-t0)
                #this loop continously makes the robot rotate until the desired angle
                if self.blueFlag == True:
                    print ("bluefound")
                    self.initalturn = True
                    break
                #if tthe rotation spots blue itll break from the loop
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)
            #publishes it to the velocity topic
        if current_ang >= target_ang:
            self.initalturn = True
        #stops if it completed the turn
    def move_towardsColour(self):
        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = -float(self.err)/200
        self.cmd_vel_pub.publish(self.move_cmd)
       #moves towards a colour it sees

    def move_away(self):
        self.movingAway = True
        target_ang = radians(120)
        current_ang = 0
        #initiates angle variables and sets the moving away to true
        print("red spotted")
        t0 = rospy.Time.now().to_sec()
        while current_ang < target_ang:
            self.move_cmd.angular.z = radians(10)
            t1 = rospy.Time.now().to_sec()
            self.cmd_vel_pub.publish(self.move_cmd)
            current_ang = radians(10) *(t1-t0)
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)
            if self.redFlag !=True:
                self.movingAway = False
                break

        if current_ang >= target_ang:
            self.movingAway = False


    #function that rotates the robot away from the red to avoid it.
    def stop_robot(self):
        "you are at road 118"
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
    #stops the robots x and z coordinates

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #connects the image from the robot into a OpenCV image
        #and transforms it into a HSV model image that is easier to work with
        lower_green = numpy.array([36, 100, 100])
        upper_green = numpy.array([70, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        #this sets up the masks for green
        lower_blue = numpy.array([120,50,50])
        upper_blue = numpy.array([150,255,255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        #this sets up the masks for blue

        lower_red = numpy.array([0,60,60])
        upper_red = numpy.array([15,255,255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        #this sets up the masks for red

        h, w, d = image.shape
       #gets the images dimensions
        Mblue = cv2.moments(mask_blue)
        Mgreen = cv2.moments(mask_green)
        Mred = cv2.moments(mask_red)
        #used to check if the masks contains any bits with those colours in
        if Mblue['m00'] > 0:
            cx = int(Mblue['m10']/Mblue['m00'])
            self.err = cx - w/2
            self.blueFlag = True
            #if the image contains blue itll set the flag to True
        else:
            self.blueFlag = False

        if Mred['m00'] > 0:
            cx = int(Mred['m10']/Mred['m00'])
            self.err = cx - w/2
            self.redFlag = True
            #checks if red is in the image
        else:
            self.redFlag = False
        
        if Mgreen['m00'] > 0:
            cx = int(Mgreen['m10']/Mgreen['m00'])
            self.err = cx - w/2
            self.greenFlag = True
            #checks if green is in the image
        else:
            self.greenFlag = False

    def run(self):
        rospy.spin()
#cv2.startWindowThread()

solver = Controller()
solver.run()
cv2.destroyAllWindows()