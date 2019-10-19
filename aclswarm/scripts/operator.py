#!/usr/bin/env python

from __future__ import division

import time
import rospy
import numpy as np

from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from acl_msgs.msg import QuadFlightMode
from aclswarm_msgs.msg import Formation
from behavior_selector.srv import MissionModeChange

NOT_FLYING = 0
FLYING = 1


class Operator:

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        self.n = len(self.vehs)
        rospy.loginfo('Using vehicles: {}'.format(self.vehs))

        # Load desired formations
        formation_group = rospy.get_param('~formation_group')
        self.formations = rospy.get_param('~{}'.format(formation_group))
        if 'adjmat' not in self.formations: # make fc if not specified
            self.formations['adjmat'] = np.ones(self.n) - np.eye(self.n)
        if self.formations['agents'] != self.n:
            rospy.logerr('Mismatch between number of vehicles ({}) '
                         'and formation group agents ({})'.format(
                            self.n, self.formations['agents']))
            rospy.signal_shutdown('Incorrect formation')
            raise rospy.ROSInitException()
        fnames = ', '.join([f['name'] for f in self.formations['formations']])
        rospy.loginfo('Using formation group \'{}\': {}'.format(formation_group, fnames))
        self.formidx = 0 # for cycling through formations in the group

        # Behavior selector and flight status management
        self.status = NOT_FLYING
        self.pubEvent = rospy.Publisher(
            '/globalflightmode', QuadFlightMode, queue_size=1, latch=True)
        self.srv = rospy.Service("change_mode", MissionModeChange, self.srvCB)

        # Publishers
        self.formationPub = rospy.Publisher(
            '/formation', Formation, queue_size=10)
        self.pubMarker = rospy.Publisher(
            "/highbay", MarkerArray, queue_size=1, latch=True)

        # Safety bounds for visualization
        self.xmax = rospy.get_param('/room_bounds/x_max', 0.0)
        self.xmin = rospy.get_param('/room_bounds/x_min', 1.0)
        self.ymax = rospy.get_param('/room_bounds/y_max', 0.0)
        self.ymin = rospy.get_param('/room_bounds/y_min', 1.0)
        self.zmin = rospy.get_param('/room_bounds/z_min', 0.0)
        self.zmax = rospy.get_param('/room_bounds/z_max', 1.0)
        self.genEnvironment()

    def sendFlightMode(self, mode):
        msg = QuadFlightMode()
        msg.header.stamp = rospy.Time.now()
        msg.mode = mode
        self.pubEvent.publish(msg)

    def srvCB(self, req):
        if req.mode == req.KILL:
            rospy.logwarn('Killing')
            self.status = NOT_FLYING
            self.sendFlightMode(QuadFlightMode.KILL)

        if req.mode == req.END and self.status == FLYING:
            rospy.logwarn('Landing')
            self.sendFlightMode(QuadFlightMode.LAND)

        if req.mode == req.START:
            if self.status == NOT_FLYING:
                rospy.logwarn('Takeoff')
                self.status = FLYING
                self.sendFlightMode(QuadFlightMode.GO)
            else:  # Already in flight
                self.pubFormation()

        return True

    def pubFormation(self):
        formation = self.formations['formations'][self.formidx]
        rospy.loginfo('\033[34;1mFormation: {}\033[0m'.format(formation['name']))

        adjmat = self.formations['adjmat']
        msg = self.buildFormationMessage(adjmat, formation)
        self.formationPub.publish(msg)

        # Cycle through formations
        self.formidx += 1
        self.formidx %= len(self.formations['formations'])

    def buildFormationMessage(self, adjmat, formation):

        # formation-related matrices
        pts = np.array(formation['points'], dtype=np.float32)
        adjmat = np.array(adjmat, dtype=np.uint8)

        msg = Formation()
        msg.header.stamp = rospy.Time.now()
        msg.name = formation['name']

        # formation points
        for pt in pts:
            msg.points.append(Point(*pt))

        # adjacency matrix for this formation group
        msg.adjmat = UInt8MultiArray()
        msg.adjmat.data = adjmat.flatten().tolist()
        msg.adjmat.layout.dim.append(MultiArrayDimension())
        msg.adjmat.layout.dim.append(MultiArrayDimension())
        msg.adjmat.layout.dim[0].label = "rows"
        msg.adjmat.layout.dim[0].size = adjmat.shape[0]
        msg.adjmat.layout.dim[0].stride = adjmat.size
        msg.adjmat.layout.dim[1].label = "cols"
        msg.adjmat.layout.dim[1].size = adjmat.shape[1]
        msg.adjmat.layout.dim[1].stride = adjmat.shape[1]

        # pre-calculated gains may have been provided, but not required
        if 'gains' in formation:
            gains = np.array(formation['gains'], dtype=np.float32)
            msg.gains = UInt8MultiArray()
            msg.gains.data = gains.flatten().tolist()
            msg.gains.layout.dim.append(MultiArrayDimension())
            msg.gains.layout.dim.append(MultiArrayDimension())
            msg.gains.layout.dim[0].label = "rows"
            msg.gains.layout.dim[0].size = self.n
            msg.gains.layout.dim[0].stride = self.n*self.n
            msg.gains.layout.dim[1].label = "cols"
            msg.gains.layout.dim[1].size = self.n
            msg.gains.layout.dim[1].stride = self.n

        return msg

    def genEnvironment(self):

        markerArray = MarkerArray()

        cx = (self.xmax + self.xmin) / 2
        cy = (self.ymax + self.ymin) / 2

        p = np.array([
                        (cx, self.ymax), # top
                        (cx, self.ymin), # bottom
                        (self.xmax, cy), # right
                        (self.xmin, cy), # left
                    ])

        THK = 0.1
        w = self.xmax - self.xmin + THK
        h = self.ymax - self.ymin + THK
        s = np.array([
                        (w, THK), # top
                        (w, THK), # bottom
                        (THK, h), # right
                        (THK, h), # left
                    ])

        for i in range(len(p)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = s[i, 0]
            marker.scale.y = s[i, 1]
            marker.scale.z = self.zmax
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = p[i, 0]
            marker.pose.position.y = p[i, 1]
            marker.pose.position.z = self.zmax / 2

            markerArray.markers.append(marker)

        self.pubMarker.publish(markerArray)


if __name__ == '__main__':
    rospy.init_node('operator')
    node = Operator()
    rospy.spin()
