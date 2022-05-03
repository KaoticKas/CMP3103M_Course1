import numpy

import cv2
from cv2 import namedWindow
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg)
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #lower_yell = numpy.array([10, 60, 70])
        #upper_yell = numpy.array([255, 255, 255])
        #mask = cv2.inRange(hsv, lower_yell, upper_yell)
        lower_blue = numpy.array([255, 0, 0])
        upper_blue = numpy.array([255, 0, 0])
        mask = cv2.inRange(image, lower_blue, upper_blue)

        cv2.bitwise_and(image, image, mask=mask)
        h, w, d = image.shape
        search_top = h
        search_bot = h
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            #self.twist.linear.x = 0.2
            #self.twist.angular.z = -float(err) / 100
            print (self.twist.angular.z)

            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", mask)
        cv2.waitKey(3)


#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()