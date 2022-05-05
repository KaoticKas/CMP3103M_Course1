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

        self.r = rospy.Rate(5)

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,queue_size=5)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)


        self.blueFlag = False
        self.redFlag = False
        self.greenFlag = False
        self.move_cmd = Twist()
        self.turnflag = False
        self.stopflag = False
        self.err = None


    def laser_callback(self, laser_data):
        print(laser_data.ranges[320])
        if laser_data.ranges[320] > 0.8:
            if(self.blueFlag !=True):   
                self.move_cmd.angular.z = 0.2
                self.cmd_vel_pub.publish(self.move_cmd)
                self.r.sleep()
            elif (self.blueFlag == True):
                self.move_cmd.angular.z = 0
                print("blue seen")
                self.move_cmd.angular.z = -float(self.err/200)
                self.move_cmd.linear.x = 0.3
                self.cmd_vel_pub.publish(self.move_cmd)
                self.r.sleep()
        else:
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)
            self.r.sleep()

    #def turn_left():
    #def turn_right():
    #def move_forwards()
    def stop_robot(self):
        self.move_cmd.angular.z = 0
        self.move_cmd.angular.x = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_green = numpy.array([36, 100, 100])
        upper_green = numpy.array([70, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        lower_blue = numpy.array([120,50,50])
        upper_blue = numpy.array([150,255,255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        lower_red = numpy.array([0,60,60])
        upper_red = numpy.array([15,255,255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        
        h, w, d = image.shape
       
        Mblue = cv2.moments(mask_blue)
        Mgreen = cv2.moments(mask_green)
        Mred = cv2.moments(mask_red)

        if Mblue['m00'] > 0:
            cx = int(Mblue['m10']/Mblue['m00'])
            cy = int(Mblue['m01']/Mblue['m00'])
            self.err = cx - w/2
            #print("blue detected")
            self.blueFlag = True
        else:
            self.blueFlag = False
            #print("no blue")


        #cv2.imshow("blue", mask_blue)
        #cv2.imshow("red", mask_red)
        #cv2.imshow("yellow", mask_green)
        #cv2.imshow("hsv", hsv)

    def move_towards():
        print("no")
        cv2.waitKey(3)
        
   

#cv2.startWindowThread()

solver = Controller()
rospy.spin()

cv2.destroyAllWindows()