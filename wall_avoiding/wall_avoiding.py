#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AvoidWall:
    def __init__(self):

        self.is_wall = False

        self.laser_msg = LaserScan()
        self.cmd_msg = Twist()

        self.laser_sub = rospy.Subscriber('/kobuki/laser/scan',LaserScan,self.laser_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    def laser_callback(self,msg):
        self.is_wall = False
        print msg.ranges
        for i in range(0,360):
            print msg.ranges[i]
            if (msg.ranges[i] < 1.0):
                self.is_wall = True
                break

    def execute(self):
        while not rospy.is_shutdown():
            if (self.is_wall):
                self.cmd_msg.linear.x = 0
                self.cmd_msg.angular.z = 1
                self.cmd_pub.publish(self.cmd_msg)
            else:
                self.cmd_msg.linear.x = 1
                self.cmd_msg.angular.z = 0
                self.cmd_pub.publish(self.cmd_msg)
            rospy.sleep(0.2)




if __name__ == '__main__':
    rospy.init_node('topics_quiz_node')
    aw = AvoidWall()
    aw.execute()