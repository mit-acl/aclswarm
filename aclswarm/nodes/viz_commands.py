#!/usr/bin/env python

from __future__ import division

import time
import rospy
import numpy as np

from acl_msgs.msg import QuadGoal, ViconState
from geometry_msgs.msg import Point, Pose, Vector3, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray


class VizCommands:

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        self.n = len(self.vehs)

        # Load desired formations
        self.viz_distcmds = rospy.get_param('~distcmds', True)
        self.viz_safecmds = rospy.get_param('~safecmds', True)
        self.viz_mesh = rospy.get_param('~mesh', True)

        # setup markers
        COLORS = {}
        COLORS['distcmd'] = (0,0,1)
        COLORS['safecmd'] = (1,0,0)
        self.create_arrow_markers(COLORS.keys(), COLORS)
        self.create_mesh_markers()

        # publishers
        self.pub_vizcmds = rospy.Publisher('viz_cmds', MarkerArray, queue_size=1)
        self.pub_vizmesh = rospy.Publisher('viz_mesh', MarkerArray, queue_size=1)

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
            if self.viz_mesh:
                self.subs[veh]['pose'] = rospy.Subscriber(
                        "/{}/vicon".format(veh), ViconState,
                        lambda msg, i=idx, v=veh: self.poseCb(msg, i, v))

    def distcmdCb(self, msg, t, veh):
        p = Point(msg.vector.x, msg.vector.y, msg.vector.z)
        self.update_arrow_marker(p, t, veh)
        self.pub_vizcmds.publish(self.markers)

    def safecmdCb(self, msg, t, veh):
        p = Point(msg.vel.x, msg.vel.y, msg.vel.z)
        self.update_arrow_marker(p, t, veh)
        self.pub_vizcmds.publish(self.markers)

    def poseCb(self, msg, vehid, vehname):
        self.markers_mesh.markers[vehid].pose = msg.pose
        self.markers_mesh.markers[vehid].header = msg.header

        self.pub_vizmesh.publish(self.markers_mesh)

    def create_mesh_markers(self):
        self.markers_mesh = MarkerArray()

        for i in range(self.n):
            m = Marker()
            m.header = None
            m.ns = 'mesh'
            m.id = i*10
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = "package://acl_sim/meshes/quadrotor/quadrotor_base.dae"
            m.action = Marker.ADD
            m.color.a = 1
            m.color.r, m.color.g, m.color.b = (1,1,1)
            m.pose.orientation.w = 1.0
            m.scale = Vector3(0.75, 0.75, 0.75)
            m.lifetime = rospy.Duration(1.0)

            self.markers_mesh.markers.append(m)

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
                m.pose.orientation.w = 1.0
                m.pose.orientation.x = 0.0
                m.pose.orientation.y = 0.0
                m.pose.orientation.z = 0.0
                if t == 'safecmd':
                    m.scale = Vector3(0.1, 0.1, 0.1) # magic numbers!
                else:
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
