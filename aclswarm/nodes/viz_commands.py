#!/usr/bin/env python

from __future__ import division

import time
import rospy
import numpy as np

from acl_msgs.msg import QuadGoal
from geometry_msgs.msg import Point, Vector3, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray


class VizCommands:

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        self.n = len(self.vehs)

        # Load desired formations
        self.viz_distcmds = rospy.get_param('~distcmds', True)
        self.viz_safecmds = rospy.get_param('~safecmds', True)

        # setup markers
        COLORS = {}
        COLORS['distcmd'] = (0,0,1)
        COLORS['safecmd'] = (1,0,0)
        self.create_arrow_markers(COLORS.keys(), COLORS)

        # publishers
        self.pub = rospy.Publisher('viz_cmds', MarkerArray, queue_size=1)

        # subscribe to relevant topics
        self.subs = {}
        for idx, veh in enumerate(self.vehs):
            self.subs[veh] = {}
            if self.viz_distcmds:
                self.subs[veh]['distcmd'] = rospy.Subscriber(
                        "/{}/distcmd".format(veh), Vector3Stamped,
                        lambda msg, v=veh: self.distcmdCb(msg, 'distcmd', v))
            if self.viz_safecmds:
                self.subs[veh]['safecmd'] = rospy.Subscriber(
                        "/{}/goal".format(veh), QuadGoal,
                        lambda msg, v=veh: self.safecmdCb(msg, 'safecmd', v))

    def distcmdCb(self, msg, t, veh):
        p = Point(msg.vector.x, msg.vector.y, msg.vector.z)
        self.update_arrow_marker(p, t, veh)
        self.pub.publish(self.markers)

    def safecmdCb(self, msg, t, veh):
        p = Point(msg.vel.x, msg.vel.y, msg.vel.z)
        self.update_arrow_marker(p, t, veh)
        self.pub.publish(self.markers)

    def create_arrow_markers(self, topics, c):
        self.markers = MarkerArray()

        for i in range(self.n):
            for t in topics:
                m = Marker()
                m.action = Marker.ADD
                m.header.frame_id = self.vehs[i]
                m.ns = t
                m.id = i*10 + topics.index(t)
                m.type = Marker.ARROW
                m.color.a = 1.0
                m.color.r, m.color.g, m.color.b = c[t]
                m.scale = Vector3(0.075, 0.075, 0.075) # magic numbers!
                m.lifetime = rospy.Duration(0.1) # this marker will be removed if not updated every period

                self.markers.markers.append(m)

    def update_arrow_marker(self, p, t, veh):
        s = 0.3 # ta-da!
        m = next((x for x in self.markers.markers if x.ns == t and x.header.frame_id == veh), None)
        if m:
            m.header.stamp = rospy.Time.now()
            m.action = Marker.MODIFY
            m.points = [Point(0,0,0), Point(s*p.x,s*p.y,s*p.z)]


if __name__ == '__main__':
    rospy.init_node('viz_commands')
    node = VizCommands()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
