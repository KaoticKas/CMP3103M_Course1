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

        self.isTurning = False

        self.initalturn = False

        self.err = None
        self.movingfromred = False

        self.laser_zoner = None
        self.laser_zonel = None
        self.laser_mid = None
        self.laser_zoner = None


    def laser_callback(self, laser_data):
        #print "hi we here again"

        lsr_right = (numpy.nansum(laser_data.ranges[0:80])/len(laser_data.ranges[0:80]))
        lsr_left = (numpy.nansum(laser_data.ranges[560:639])/len(laser_data.ranges[560:639]))
        lz = (numpy.nansum(laser_data.ranges[0:280])/len(laser_data.ranges[0:280]))
        rz = (numpy.nansum(laser_data.ranges[400:639])/len(laser_data.ranges[400:639]))
        lsr_mid = (numpy.nansum(laser_data.ranges[290:350])/len(laser_data.ranges[290:350]))

        if self.initalturn !=True:
            self.initial_turn()
        if (lz > 0.7 and lsr_mid > 0.7 and rz > 0.7) and (self.blueFlag !=False or self.greenFlag !=False) and self.redFlag !=True:
            self.move_towardsColour()
        elif (lz >0.7 and lsr_mid > 0.7 and rz > 0.7) and self.redFlag != True:
            self.move_forwards()
            print ("movin forwards")
        else:
            if self.redFlag == True:
                self.move_away()
            elif (numpy.nansum(laser_data.ranges[0:80])/len(laser_data.ranges[0:80])) > (numpy.nansum(laser_data.ranges[559:639])/len(laser_data.ranges[559:639]))  and self.isTurning != True:
                self.stop_robot()
                self.turn_right()
                print ("We turning right")
            elif (numpy.nansum(laser_data.ranges[559:639])/len(laser_data.ranges[559:639]))> (numpy.nansum(laser_data.ranges[0:80])/len(laser_data.ranges[0:80])) and self.isTurning !=True:
                self.stop_robot()
                self.turn_left()
                print ("We turning left")

            else:
                #maybe add a while loop to turn until an angle is reached, idk
                self.stop_robot()
                self.turn_rightH()
                print ("just why")
        self.isTurning = True


    def initial_turn(self):
        target_ang = radians(360)
        current_ang = 0
        if self.initalturn != True:

            t0 = rospy.Time.now().to_sec()
            while current_ang < target_ang:
                self.move_cmd.angular.z = radians(20)
                t1 = rospy.Time.now().to_sec()
                self.cmd_vel_pub.publish(self.move_cmd)
                current_ang = radians(10) *(t1-t0)
                if self.blueFlag == True:
                    print ("bluefound")
                    self.initalturn = True
                    break
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)
        if current_ang >= target_ang:
            self.initalturn = True

    def move_forwards(self):

        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
        #self.r.sleep()
        print ("you are at road 94")


    def move_towardsColour(self):
        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = -float(self.err)/200
        #self.r.sleep()
        self.cmd_vel_pub.publish(self.move_cmd)

    def move_away(self):
        self.movingfromred = True
        target_ang = radians(180)
        current_ang = 0
        print("red spotted")
        t0 = rospy.Time.now().to_sec()
        while current_ang < target_ang:
            self.move_cmd.angular.z = radians(10)
            t1 = rospy.Time.now().to_sec()
            self.cmd_vel_pub.publish(self.move_cmd)
            current_ang = radians(10) *(t1-t0)
            self.move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.move_cmd)
        self.movingfromred = True 


    def stop_robot(self):
        "you are at road 118"
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        #self.r.sleep()


    def turn_left(self):
        self.isTurning = True
        self.move_cmd.angular.z = -0.5
        print ("you are at road 125")
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()


    def turn_right(self):
        self.isTurning = True
        self.move_cmd.angular.z = 0.5
        print ("you are at road 132")
        self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()


    
    def turn_rightH(self):
        self.isTurning = True
        for x in range(0,10):
            self.move_cmd.angular.z = radians(35)
            print ("you are at road 999")
            self.cmd_vel_pub.publish(self.move_cmd)
        self.r.sleep()
        self.isTurning = False


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
        cv2.imshow("red", mask_red)
        #cv2.imshow("yellow", mask_green)
        #cv2.imshow("hsv", hsv)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()
#cv2.startWindowThread()

solver = Controller()
solver.run()
cv2.destroyAllWindows()