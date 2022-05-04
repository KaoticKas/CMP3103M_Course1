import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import radians

class laser_reader:

    def __init__(self):
        self.laser_sub = rospy.Subscriber("/scan",
                                          LaserScan,
                                          self.callback)

    def callback(self, data):
        if data.ranges[320] < 1.0:# the laser has a range of 640 points from left to right
            # it takes the middle point and checks if its 1m behind
            t = Twist()
            t.angular.z = 1.0
            self.publisher.publish(t)

ic = laser_reader()
rospy.init_node('laser_reader')
rospy.spin()