#!/usr/bin/env python

from __future__ import division

import time
import rospy
import numpy as np

from aclswarm_msgs.msg import Formation
from acl_msgs.msg import QuadGoal, ViconState
from geometry_msgs.msg import Point, Pose, Vector3, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray

from aclswarm.assignment import align

class VizCommands:

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        self.n = len(self.vehs)

        # keep track of what the current formation points are
        self.formpts = None

        # Load desired formations
        self.viz_distcmds = rospy.get_param('~distcmds', True)
        self.viz_safecmds = rospy.get_param('~safecmds', True)
        self.viz_mesh = rospy.get_param('~mesh', True)

        # setup markers
        self.markers_distcmd = self.create_arrow_markers('distcmd', (0,0,1))
        self.markers_safecmd = self.create_arrow_markers('safecmd', (1,0,0))
        self.markers_aligned = self.create_sphere_markers('aligned', (0.1,0.1,0.1))
        self.markers_mesh = self.create_mesh_markers()

        # publishers
        self.pub_vizdistcmd = rospy.Publisher('viz_dist_cmd', MarkerArray, queue_size=1)
        self.pub_vizsafecmd = rospy.Publisher('viz_safe_cmd', MarkerArray, queue_size=1)
        self.pub_vizaligned = rospy.Publisher('viz_central_alignment', MarkerArray, queue_size=1)
        self.pub_vizmesh = rospy.Publisher('viz_mesh', MarkerArray, queue_size=1)

        # keep track of where everyone is
        self.poses = [None]*self.n

        # subscribe to relevant topics
        self.subs = {}
        for idx, veh in enumerate(self.vehs):
            self.subs[veh] = {}
            if self.viz_distcmds:
                self.subs[veh]['distcmd'] = rospy.Subscriber(
                        "/{}/distcmd".format(veh), Vector3Stamped,
                        lambda msg, v=veh: self.distcmdCb(msg, 'distcmd', v),
                        queue_size=1)
            if self.viz_safecmds:
                self.subs[veh]['safecmd'] = rospy.Subscriber(
                        "/{}/goal".format(veh), QuadGoal,
                        lambda msg, v=veh: self.safecmdCb(msg, 'safecmd', v),
                        queue_size=1)
            if self.viz_mesh:
                self.subs[veh]['pose'] = rospy.Subscriber(
                        "/{}/vicon".format(veh), ViconState,
                        lambda msg, i=idx, v=veh: self.poseCb(msg, i, v),
                        queue_size=1)

        self.sub_formation = rospy.Subscriber('/formation', Formation,
                                                self.formationCb, queue_size=1)

        # timers
        self.tim_vizaligned = rospy.Timer(rospy.Duration(0.2), self.vizAlignedCb)

    def distcmdCb(self, msg, t, veh):
        p = Point(msg.vector.x, msg.vector.y, msg.vector.z)
        self.update_arrow_marker(self.markers_distcmd, p, t, veh)
        self.pub_vizdistcmd.publish(self.markers_distcmd)

    def safecmdCb(self, msg, t, veh):
        p = Point(msg.vel.x, msg.vel.y, msg.vel.z)
        self.update_arrow_marker(self.markers_safecmd, p, t, veh)
        self.pub_vizsafecmd.publish(self.markers_safecmd)

    def poseCb(self, msg, vehid, vehname):
        # update mesh marker
        self.markers_mesh.markers[vehid].pose = msg.pose
        self.markers_mesh.markers[vehid].header = msg.header
        self.pub_vizmesh.publish(self.markers_mesh)

        self.poses[vehid] = msg

    def formationCb(self, msg):
        pts = np.zeros((3,self.n), dtype=np.float32)
        for (i,pt) in enumerate(msg.points):
            pts[:,i] = (pt.x, pt.y, pt.z)
        self.formpts = pts

    def vizAlignedCb(self, event=None):
        # do we have formation points yet?
        if self.formpts is None: return

        # have we received everyone's pose?
        if None in self.poses: return

        # construct matrix of swarm positions (3xn)
        q = np.array([(msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z)
                            for msg in self.poses]).T
        p = self.formpts

        # visualize current alignment
        pa = align(q, p)
        stamp = rospy.Time.now()
        for i in range(self.n):
            self.markers_aligned.markers[i].header.stamp = stamp
            self.markers_aligned.markers[i].header.frame_id = self.poses[0].header.frame_id
            self.markers_aligned.markers[i].pose.position.x = pa[0,i]
            self.markers_aligned.markers[i].pose.position.y = pa[1,i]
            self.markers_aligned.markers[i].pose.position.z = pa[2,i]
        self.pub_vizaligned.publish(self.markers_aligned)

    def create_mesh_markers(self):
        markers = MarkerArray()

        for i in range(self.n):
            m = Marker()
            m.header.frame_id = self.vehs[i]
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

            markers.markers.append(m)

        return markers

    def create_sphere_markers(self, ns, color):
        markers = MarkerArray()

        for i in range(self.n):
            m = Marker()
            m.ns = ns
            m.id = i*10
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.color.a = 1
            m.color.r, m.color.g, m.color.b = color
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (0.75, 0.75, 0.75)
            m.lifetime = rospy.Duration(1.0)

            markers.markers.append(m)

        return markers

    def create_arrow_markers(self, mtype, color):
        markers = MarkerArray()

        for i in range(self.n):
            m = Marker()
            m.action = Marker.ADD
            m.header.frame_id = self.vehs[i]
            m.ns = mtype
            m.id = i*10
            m.type = Marker.ARROW
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = color
            m.pose.orientation.w = 1.0
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.scale = Vector3(0.1, 0.1, 0.1) # magic numbers!
            m.lifetime = rospy.Duration(1.0)

            markers.markers.append(m)

        return markers

    def update_arrow_marker(self, markers, p, t, veh):
        s = 0.5 # ta-da!
        m = next((x for x in markers.markers if x.ns == t and x.header.frame_id == veh), None)
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
