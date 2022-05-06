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
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.move_cmd = Twist()

        self.blueFlag = False
        self.redFlag = False
        self.greenFlag = False

        self.turnflag = False
        self.stopflag = False
        self.initalturnflag = False
        self.err = None


    def laser_callback(self, laser_data):

        if self.initalturnflag !=True:
            self.initial_turn()

        while numpy.nansum(laser_data.ranges[290:350])/len(laser_data.ranges[290:350]) > 1:
            if self.blueFlag !=True and self.greenFlag !=True and self.redFlag != True:
                self.move_forwards()
                break
            elif self.blueFlag == True:
                self.move_towardsColour()
                break
            elif self.greenFlag == True:
                hi =32
                break
            elif self.redFlag == True:
                fu = 32
                break
        while numpy.nansum(laser_data.ranges[290:350])/len(laser_data.ranges[290:350]) < 0.7:
            self.stop_robot()
            if (numpy.nansum(laser_data.ranges[0:180])/len(laser_data.ranges[0:180])) > laser_data.ranges[320]:
                print("Turning left")
                self.turn_left()
                self.r.sleep()
                break
            elif (numpy.nansum(laser_data.ranges[450:639])/len(laser_data.ranges[450:639])) > laser_data.ranges[320]:
                print("Turning right")
                self.turn_right()
                self.r.sleep()
                break


        #else:
            #self.turnflag =False
            #self.stopflag = False
            #self.move_forwards()
            #print("im here 77")
            


    def initial_turn(self):
        target_ang = radians(360)
        current_ang = 0
        if self.initalturnflag != True:

            t0 = rospy.Time.now().to_sec()
            while current_ang < target_ang:
                self.move_cmd.angular.z = radians(10)
                t1 = rospy.Time.now().to_sec()
                self.cmd_vel_pub.publish(self.move_cmd)
                current_ang = radians(10) *(t1-t0)
                print(current_ang)
                if self.blueFlag == True:
                    print ("bluefound")
                    self.initalturnflag = True
                    break
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)
        if current_ang >= target_ang:
            self.initalturnflag = True

    def move_forwards(self):
        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()


    def move_towardsColour(self):
        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = -float(self.err)/200
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()

    def move_away(self):
        target_ang = radians(180)
        current_ang = 0
        t0 = rospy.Time.now().to_sec()
        while current_ang < target_ang:
            self.move_cmd.angular.z = radians(10)
            t1 = rospy.Time.now().to_sec()
            self.cmd_vel_pub.publish(self.move_cmd)
            current_ang = radians(10) *(t1-t0)
            print(current_ang)
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)

    def stop_robot(self):
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()


    def turn_left(self):
        self.move_cmd.angular.z = -0.3
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()
        self.stop_robot()

    def turn_right(self):
        self.move_cmd.angular.z = 0.3
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()
        self.stop_robot()


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
            self.err = cx - w/2
            self.blueFlag = True
        else:
            self.blueFlag = False

        if Mred['m00'] > 0:
            cx = int(Mred['m10']/Mred['m00'])
            self.err = cx - w/2
            self.redFlag = True
        else:
            self.redFlag = False
        
        if Mgreen['m00'] > 0:
            cx = int(Mgreen['m10']/Mgreen['m00'])
            self.err = cx - w/2
            self.greenFlag = True
        else:
            self.greenFlag = False




        #cv2.imshow("blue", mask_blue)
        #cv2.imshow("red", mask_red)
        #cv2.imshow("yellow", mask_green)
        #cv2.imshow("hsv", hsv)
        cv2.waitKey(3)

#cv2.startWindowThread()

solver = Controller()
rospy.spin()

cv2.destroyAllWindows()