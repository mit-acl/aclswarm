#!/usr/bin/env python

from __future__ import division

import sys, time
import rospy
import numpy as np

from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from snapstack_msgs.msg import QuadFlightMode
from aclswarm_msgs.msg import Formation
from behavior_selector.srv import MissionModeChange

from aclswarm.control import createGainMatrix
from aclswarm.assignment import find_optimal_assignment

NOT_FLYING = 0
FLYING = 1


class Operator:

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        self.n = len(self.vehs)
        rospy.loginfo('Using vehicles: {}'.format(self.vehs))

        # Load desired formations
        self.send_gains = rospy.get_param('~send_gains', False)
        formation_group = rospy.get_param('~formation_group')
        self.formations = rospy.get_param('~{}'.format(formation_group))
        self.manageAdjmat()
        if self.formations['agents'] != self.n:
            rospy.logerr('Mismatch between number of vehicles ({}) '
                         'and formation group agents ({})'.format(
                            self.n, self.formations['agents']))
            rospy.signal_shutdown('Incorrect formation')
            raise rospy.ROSInitException()
        fnames = ', '.join([f['name'] for f in self.formations['formations']])
        rospy.loginfo('Using formation group \'{}\': {}'.format(formation_group, fnames))
        self.formidx = -1 # for cycling through formations in the group

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

        # Handle centralized assignment --- only for comparison
        self.central_assignment = rospy.get_param('~central_assignment', False)
        self.assignment_dt = rospy.get_param('~central_assignment_dt', 0.75)
        if self.central_assignment:
            rospy.logwarn('Generating centralized assignment')

            # last assignment
            self.P = None

            # ground truth state of each vehicle
            self.poses = [None]*self.n
            for idx,veh in enumerate(self.vehs):
                rospy.Subscriber('/{}/world'.format(veh), PoseStamped,
                        lambda msg, i=idx: self.poseCb(msg, i), queue_size=1)

            self.tim_sendassign = rospy.Timer(rospy.Duration(self.assignment_dt),
                                                self.sendAssignmentCb)
            self.pub_assignment = rospy.Publisher('/central_assignment',
                                                UInt8MultiArray, queue_size=1)

        # Safety bounds for visualization
        self.xmax = rospy.get_param('/room_bounds/x_max', 0.0)
        self.xmin = rospy.get_param('/room_bounds/x_min', 1.0)
        self.ymax = rospy.get_param('/room_bounds/y_max', 0.0)
        self.ymin = rospy.get_param('/room_bounds/y_min', 1.0)
        self.zmin = rospy.get_param('/room_bounds/z_min', 0.0)
        self.zmax = rospy.get_param('/room_bounds/z_max', 1.0)
        self.genEnvironment()

    def manageAdjmat(self):
        """Manage adjacency natrix
        Each formation within a group can have its own adjacency matrix.
        If a global adjacency matrix is supplied, that one will override
        each of the (optionally in this case) supplied formation adjmat.
        """

        # check if valid, global adjmat provided
        has_global_adjmat = 'adjmat' in self.formations

        # make sure each formation has a valid adjmat
        for formation in self.formations['formations']:

            if has_global_adjmat:
                # set this formation's adjmat to the globally provided adjmat
                formation['adjmat'] = self.formations['adjmat']

            if ('adjmat' not in formation # make fc if not specified
                    or type(formation['adjmat']) != list):
                rospy.loginfo("Using fully connected formation graph for "
                            "\033[34;1m{}\033[0m".format(formation['name']))
                formation['adjmat'] = np.ones(self.n) - np.eye(self.n)


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
        # Cycle through formations (starts at -1)
        self.formidx += 1
        self.formidx %= len(self.formations['formations'])

        formation = self.formations['formations'][self.formidx]
        rospy.loginfo('\033[34;1mFormation: {}\033[0m'.format(formation['name']))

        msg = self.buildFormationMessage(formation)
        self.formationPub.publish(msg)

        # if we are a centralized coordinator, reset the assignment
        if self.central_assignment:
            self.P = None

    def getPoints(self, formation):
        scale = float(formation['scale']) if 'scale' in formation else 1.0
        return scale * np.array(formation['points'], dtype=np.float32)

    def buildFormationMessage(self, formation):

        # formation-related matrices
        pts = self.getPoints(formation)
        adjmat = np.array(formation['adjmat'], dtype=np.uint8)

        msg = Formation()
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

        # should we include formation gains in our message?
        if self.send_gains:

            # pre-calculated gains may have been provided, but not required.
            # Store the gains so we do not need to redo work.
            if 'gains' not in formation:
                formation['gains'] = createGainMatrix(adjmat, pts, method='original')

                # # Print gain matrix for easy copying into formations.yaml
                # np.set_printoptions(linewidth=500, threshold=sys.maxsize)
                # print(np.array2string(formation['gains'], separator=', ',
                #             formatter={'float_kind':lambda x: "% 0.4f" % x}))

            A = formation['gains']

            # pack up the gains into the message
            gains = np.array(A, dtype=np.float32)
            msg.gains = UInt8MultiArray()
            msg.gains.data = gains.flatten().tolist()
            msg.gains.layout.dim.append(MultiArrayDimension())
            msg.gains.layout.dim.append(MultiArrayDimension())
            msg.gains.layout.dim[0].label = "rows"
            msg.gains.layout.dim[0].size = gains.shape[0]
            msg.gains.layout.dim[0].stride = gains.size
            msg.gains.layout.dim[1].label = "cols"
            msg.gains.layout.dim[1].size = gains.shape[1]
            msg.gains.layout.dim[1].stride = gains.shape[1]

        msg.header.stamp = rospy.Time.now()
        return msg

    def poseCb(self, msg, i):
        # n.b., this is only used when sending centralized assignments
        # n.b., we are using the index of this vehicle in the /vehs list
        # so that we create `q` with the correct order.
        self.poses[i] = msg

    def sendAssignmentCb(self, event=None):
        """
        This is only used for comparison to distributed ACLswarm assignment.

        Note: Although this callback has its own period, assignments will
                note take effect until every `autoauction_dt`.
                See coordination.launch.
        """

        # do we have all the data we need?
        if None in self.poses or self.formidx == -1: return

        # construct matrix of swarm positions (3xn)
        q = np.array([(msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z)
                            for msg in self.poses]).T
        p = self.getPoints(self.formations['formations'][self.formidx]).T

        # use global swarm state to find optimal assignment via Hungarian
        self.P, _ = find_optimal_assignment(q, p, last=self.P) # for n = 15, takes 5-10 ms

        # Publish to the swarm
        msg = UInt8MultiArray()
        msg.data = self.P
        self.pub_assignment.publish(msg)

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
