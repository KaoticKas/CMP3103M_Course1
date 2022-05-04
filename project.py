import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


blueflag = False

class Solver:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def laser_callback(self, data):
        if data.ranges[320] < 1.0:# the laser has a range of 640 points from left to right
            # it takes the middle point and checks if its 1m behind
            if blueflag == True:
                t = Twist()
                t.angular.z = 1.0
                self.cmd_vel_pub.publish(t)


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

        satthresh = cv2.inRange(hsv,numpy.array((0,150,50)),numpy.array((255,255,255)))

        greenbit =cv2.bitwise_and(image, image, mask=mask_green)
        bluebit =cv2.bitwise_and(image, image, mask=mask_blue)
        redbit =cv2.bitwise_and(image, image, mask=mask_red)
        h, w, d = image.shape
        search_top = h
        search_bot = h
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0
        Mblue = cv2.moments(mask_blue)
        #if M['m00'] > 0:
            #cx = int(M['m10']/M['m00'])
            #cy = int(M['m01']/M['m00'])
            #cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            #err = cx - w/2
            #self.twist.linear.x = 0.2
            #self.twist.angular.z = -float(err) / 100
            #print (self.twist.angular.z)

            #self.cmd_vel_pub.publish(self.twist)
        #print("hi mark")

        #print(cv2.mean(hsv[:, :, 0], mask = satthresh)[0])
        #print(cv2.mean(hsv[:, :, 1], mask = satthresh)[0])

        if Mblue['m00'] > 0:
            print("blue detected")
            blueflag = True
        else:
            blueflag = False
            print("no blue")


        cv2.imshow("blue", mask_blue)
        cv2.imshow("red", mask_red)
        cv2.imshow("yellow", mask_green)
        cv2.imshow("hsv", hsv)

        cv2.waitKey(3)
        


#cv2.startWindowThread()
rospy.init_node('solver')
solver = Solver()
rospy.spin()

cv2.destroyAllWindows()