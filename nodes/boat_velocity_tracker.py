#!/usr/bin/python

import rospy
from geometry_msgs.msg import Point, Pose, Vector3
import math
from kalman_filter import Kalman
from social_navigation_layers.msg import Boat, Boats
from kalman_filter import Kalman

def subtract(v1, v2):
    return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)

def scale(v, s):
    v.x *= s
    v.y *= s
    v.z *= s

class BoatEstimate(object):
    def __init__(self, msg, stamp):
        self.det_boat = msg
        self.stamp = stamp
        self.k = Kalman()

    def update(self, msg, stamp):
        last = self.det_boat
        last_stamp = self.stamp
        self.det_boat = msg
        self.stamp = stamp

        ivel = subtract(self.det_boat.pose.position, last.pose.position)
        if ((self.stamp - last_stamp).to_sec() != 0.0):
            time = (self.stamp - last_stamp).to_sec()
            scale(ivel, 1.0 / time)

        self.k.update([ivel.x, ivel.y, ivel.z])

    def id(self):
        return self.det_boat.id

    def velocity(self):
        k = self.k.values()
        if k == None:
            return Vector3()
        v = Vector3(k[0], k[1], k[2])
        return v

    def get_boat(self):
        b = Boat()
        b.id = self.id()
        b.lidar_index = self.det_boat.lidar_index
        b.pose = self.det_boat.pose
        b.velocity = self.velocity()
        b.size = self.det_boat.size
        return b


class VelocityTracker(object):
    def __init__(self):
        self.boats = {}
        self.boat_received_time = rospy.get_time()
        self.TIMEOUT = rospy.get_param('~timeout', 3.0)
        self.sub = rospy.Subscriber('boats_detected',
                                    Boats,
                                    self.bm_cb)
        self.bpub = rospy.Publisher('/boats',
                                    Boats,
                                    queue_size=10)

    def bm_cb(self, msg):
        self.keep_dict = {}
        self.boat_received_time = rospy.get_time()
        for bm in msg.boats:
            self.keep_dict[bm.id] = bm
            if bm.id in self.boats:
                self.boats[bm.id].update(bm, msg.header.stamp)
            else:
                b = BoatEstimate(bm, msg.header.stamp)
                self.boats[bm.id] = b

    def spin(self):
        rate = rospy.Rate(0.62)
        while not rospy.is_shutdown():
            self.publish()
            for boat in self.boats.keys():
                if boat not in self.keep_dict:
                    del self.boats[boat]
            if (rospy.get_time() - self.boat_received_time) > self.TIMEOUT:
                self.boats = {}
            rate.sleep()

    def publish(self):
        bl = Boats()
        bl.header.frame_id = "map"

        for b in self.boats.values():
            boat = b.get_boat()
            bl.boats.append(boat)

        self.bpub.publish(bl)

rospy.init_node("boat_velocity_tracker")
vt = VelocityTracker()
vt.spin()
