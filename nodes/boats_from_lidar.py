#!/usr/bin/env pythonboats_from_lidar
'''
This node takes data from the lidar and uses it to fill the /boats topic
'''

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf import transformations
from math import radians, degrees


class Node():
    def __init__(self):

        rospy.Subscriber("imu/data", Imu, self.callback)
        self.imu_msg = None
        self.pub = rospy.Publisher("is_boat_horizontal", Bool, queue_size=1)
        rospy.loginfo("Checking for horizontal")
        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.calc_and_publish()
            loop_rate.sleep()

    def callback(self, msg):
        self.imu_msg = msg

    def calc_and_publish(self):
        if not self.imu_msg:
            return

        quat = self.imu_msg.orientation
        euler = transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        X, Y, Z = 0, 1, 2  # NOQA
        rospy.logdebug("Dory pitch (Y rotation) is {:2f}deg".format(degrees(euler[Y])))
        horizontal = abs(euler[Y]) < radians(10)
        self.pub.publish(horizontal)
        self.imu_msg = None  # Don't process old messages again


if __name__ == '__main__':
    rospy.init_node('is_boat_horizontal')

    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
