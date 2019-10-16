#!/usr/bin/env python

from __future__ import division

import time
import rospy
import numpy as np

from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from acl_msgs.msg import MissionMode, QuadFlightMode
from behavior_selector.srv import MissionModeChange
from pgswarm_msgs.msg import MutualVOTransforms

NOT_FLYING = 0
FLYING = 1


class Operator:

    def __init__(self):

        # Behavior selector trackers
        self.status = NOT_FLYING
        self.pubEvent = rospy.Publisher(
            "globalflightmode", QuadFlightMode, queue_size=1, latch=True)
        self.flightevent = QuadFlightMode()

        # Publisher and tracker
        self.pointPub = rospy.Publisher(
            '/formationpoints', PoseArray, queue_size=10)
        self.adjPub = rospy.Publisher(
            '/adjacencymatrix', Int32MultiArray, queue_size=1)
        self.pubMarker = rospy.Publisher(
            "/highbay", MarkerArray, queue_size=1, latch=True)
        self.i = 0

        # Create formation  and messages
        vehs = rospy.get_param('/vehs')
        count = len(vehs)
        self.formation_names = rospy.get_param(
            '~{}_agent_formation_names'.format(count))
        self.formation_points = rospy.get_param(
            '~{}_agent_formation_points'.format(count))
        self.formation_msgs = self.createFormationMsgs(self.formation_points)

        # Publish initial adjacency matrix and formation
        adj_matrix = np.array(rospy.get_param(
            '~{}_agent_adjacency_matrix'.format(count)), dtype=np.int32)
        self.adj_msg = Int32MultiArray(data=adj_matrix.flatten())
        self.adj_msg.layout.dim.append(MultiArrayDimension())
        self.adj_msg.layout.dim.append(MultiArrayDimension())
        self.adj_msg.layout.dim[0].label = "rows"
        self.adj_msg.layout.dim[0].size = count
        self.adj_msg.layout.dim[0].stride = count*count
        self.adj_msg.layout.dim[1].label = "cols"
        self.adj_msg.layout.dim[1].size = count
        self.adj_msg.layout.dim[1].stride = count

        # Publish vehicle information
        rospy.loginfo('Using vehicles: {}'.format(vehs))

        # Safety bounds for visualization
        self.xmax = rospy.get_param('/room_bounds/x_max', 0.0)
        self.xmin = rospy.get_param('/room_bounds/x_min', -8.0)
        self.ymax = rospy.get_param('/room_bounds/y_max', 3.0)
        self.ymin = rospy.get_param('/room_bounds/y_min', -3.0)

        # For tracking vehicles (mutual VO)
        self.poses = {v: Pose() for v in vehs}
        self.vehs = vehs
        self.transform_publishers = {v: rospy.Publisher(
            '/' + v + '/transforms', MutualVOTransforms, queue_size=1) for v in self.vehs}

        rospy.sleep(1)
        self.genEnvironment()

    def sendEvent(self):
        self.flightevent.header.stamp = rospy.get_rostime()
        self.pubEvent.publish(self.flightevent)

    def change_mode(self, req):
        if req.mode == req.KILL:
            self.status = NOT_FLYING
            self.flightevent.mode = self.flightevent.KILL
            self.sendEvent()

        if req.mode == req.END and self.status == FLYING:
            self.flightevent.mode = self.flightevent.LAND
            self.sendEvent()

        if req.mode == req.START:
            if self.status == NOT_FLYING:
                self.status = FLYING
                self.flightevent.mode = self.flightevent.GO
                self.sendEvent()
            else:  # Already in flight
                self.pubFormation()

    def trackerCB(self, data, veh):
        # Update vehicle pose
        self.poses[veh] = data.pose

    def srvCB(self, req):
        self.change_mode(req)
        return True

    def mutualViconSrvCB(self, req):
        """ Mutual VICON service handler
        Provides a relative measurement between vehicles (mimicking mutual VO)
        and sends the relevant transforms to each vehicle's VehicleTracker.
        """
        for veh_i in self.vehs:
            for veh_j in self.vehs:
                if veh_i == veh_j:
                    continue

                msg = MutualVOTransforms()
                msg.header.stamp = rospy.Time.now()
                msg.veh.data = veh_j

                # TODO: send relative orientations
                msg.curr_transform.translation = Vector3(
                    x=self.poses[veh_i].position.x,
                    y=self.poses[veh_i].position.y,
                    z=self.poses[veh_i].position.z)
                msg.curr_transform.rotation = Quaternion(x=0, y=0, z=0, w=1)

                msg.nbr_transform.translation = Vector3(
                    x=self.poses[veh_j].position.x,
                    y=self.poses[veh_j].position.y,
                    z=self.poses[veh_j].position.z)
                msg.nbr_transform.rotation = Quaternion(x=0, y=0, z=0, w=1)

                msg.rel_transform.translation = Vector3(
                    x=self.poses[veh_j].position.x -
                    self.poses[veh_i].position.x,
                    y=self.poses[veh_j].position.y -
                    self.poses[veh_i].position.y,
                    z=self.poses[veh_j].position.z - self.poses[veh_i].position.z)
                msg.rel_transform.rotation = Quaternion(x=0, y=0, z=0, w=1)

                self.transform_publishers[veh_i].publish(msg)

        return TriggerResponse(True, "")

    def pubFormation(self):
        print('Formation {}'.format(self.formation_names[self.i]))
        self.adjPub.publish(self.adj_msg)
        rospy.sleep(1.0)
        self.pointPub.publish(self.formation_msgs[self.i])

        self.i += 1
        self.i %= len(self.formation_msgs)

    def createFormationMsgs(self, formation_points):
        msgs = []
        for formation in formation_points:
            formationArray = PoseArray()

            for point in formation:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = point[2]
                formationArray.poses.append(pose)

            msgs.append(formationArray)
        return msgs

    def genEnvironment(self):

        markerArray = MarkerArray()

        dx = self.xmax - self.xmin
        dy = self.ymax - self.ymin

        diffx = self.xmax + self.xmin

        p = np.array([(diffx / 2, self.ymax), (diffx / 2, self.ymin),
                      (self.xmax + 0.5, 0), (self.xmin - 0.5, 0)])
        s = np.array([(dx + 1, 0.1), (dx + 1, 0.1), (0.1, dy), (0.1, dy)])

        for i in range(len(p)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = s[i, 0]
            marker.scale.y = s[i, 1]
            marker.scale.z = 3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = p[i, 0]
            marker.pose.position.y = p[i, 1]
            marker.pose.position.z = 1.5

            markerArray.markers.append(marker)

        self.pubMarker.publish(markerArray)


def startNode():
    c = Operator()
    s = rospy.Service("change_mode", MissionModeChange, c.srvCB)
    m = rospy.Service("mutualvicon", Trigger, c.mutualViconSrvCB)
    for v in c.vehs:
        rospy.Subscriber("/" + v + "/world", PoseStamped,
                         callback=c.trackerCB, callback_args=v)
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('operator')
    print("Starting operator node...")
    startNode()