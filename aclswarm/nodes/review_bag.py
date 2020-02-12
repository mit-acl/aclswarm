#!/usr/bin/env python

from __future__ import division

import argparse
import time
import csv
from collections import deque

import rospy
import numpy as np; np.set_printoptions(linewidth=500)

import tf2_ros

from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TransformStamped
from aclswarm_msgs.msg import SafetyStatus, Formation
from behavior_selector.srv import MissionModeChange
from snapstack_msgs.msg import QuadGoal, State as SnapState

class State:
    WAITING_ON_FORMATION = 3
    WAITING_ON_ASSIGNMENT = 4
    FLYING = 5
    IN_FORMATION = 6
    GRIDLOCK = 7
    COMPLETE = 8


def S(state):
    """Stringify state
    """
    if state == State.WAITING_ON_FORMATION: return "WAITING_ON_FORMATION"
    if state == State.WAITING_ON_ASSIGNMENT: return "WAITING_ON_ASSIGNMENT"
    if state == State.FLYING: return "FLYING"
    if state == State.IN_FORMATION: return "IN_FORMATION"
    if state == State.GRIDLOCK: return "GRIDLOCK"
    if state == State.COMPLETE: return "\033[32;1mCOMPLETE\033[0m"


class Reviewer:
    # for each predicate, how much data should be averaged over?
    BUFFER_SECONDS = 1

    # times for state machine---in units of seconds
    CONVERGED_WAIT = 0
    FORMATION_WAIT = 1

    # thresholds
    ZERO_POS_THR = 0.05 # m
    ORIG_ZERO_VEL_THR = 0.125 # m/s
    AVG_ACTIVE_CA_THR = 0.95 # percent

    NUM_FORMATIONS_IN_TRIAL = 3

    def __init__(self):

        # find out which vehicles are in the swarm
        topics = [t for t,tt in rospy.get_published_topics()]
        self.vehs = [t.split('/')[1] for t in topics if 'assignment' in t]

        # for relaying poses to the tf tree
        self.tf = tf2_ros.TransformBroadcaster()

        # for signal smoothing
        self.alpha =  0.98

        self.log = {}
        self.vstates = {}
        self.voriggoal = {}
        self.vsafegoal = {}
        self.vstatus = {}

        for idx, veh in enumerate(self.vehs):
            # ground truth state of each vehicle
            rospy.Subscriber('/{}/world'.format(veh), PoseStamped,
                    lambda msg, v=veh: self.poseCb(msg, v), queue_size=1)
            # imu/vio state of each vehicle
            rospy.Subscriber('/{}/state_throttle'.format(veh), SnapState,
                    lambda msg, v=veh: self.stateCb(msg, v), queue_size=1)
            # the desired velocity goal from the distributed motion planner
            rospy.Subscriber('/{}/distcmd_throttle'.format(veh), Vector3Stamped,
                    lambda msg, v=veh: self.origGoalCb(msg, v), queue_size=1)
            # the safe, collision-free version of the motion planner vel goal
            rospy.Subscriber('/{}/goal_throttle'.format(veh), QuadGoal,
                    lambda msg, v=veh: self.safeGoalCb(msg, v), queue_size=1)
            # status flags from Safety goal (i.e., collision avoidance active)
            rospy.Subscriber('/{}/safety/status_throttle'.format(veh), SafetyStatus,
                    lambda msg, v=veh: self.statusCb(msg, v), queue_size=1)

            # we only need one subscriber for the following
            if idx == 0:
                # an assignment was generated
                rospy.Subscriber('/{}/assignment'.format(veh), UInt8MultiArray,
                    lambda msg, v=veh: self.assignmentCb(msg, v), queue_size=1)

        rospy.Subscriber('/formation', Formation, self.formationCb, queue_size=1)

        # initialize state machine variables
        self.state = State.WAITING_ON_FORMATION
        self.last_state = None
        self.timer_ticks = -1
        self.curr_formation_idx = -1
        self.received_formation = False
        self.received_assignment = False
        self.tick_rate = 50
        self.is_logging = False

        # ring buffers for checking windowed signal averages
        self.BUFFLEN = self.BUFFER_SECONDS * self.tick_rate
        self.buffers = {}

        rate = rospy.Rate(self.tick_rate)
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()

    #
    # ROS callbacks
    #

    def poseCb(self, msg, veh):
        self.vstates[veh] = msg

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = veh
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w
        self.tf.sendTransform(t)

    def stateCb(self, msg, veh):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = veh + "/imu"
        t.transform.translation.x = msg.pos.x
        t.transform.translation.y = msg.pos.y
        t.transform.translation.z = msg.pos.z
        t.transform.rotation.x = msg.quat.x
        t.transform.rotation.y = msg.quat.y
        t.transform.rotation.z = msg.quat.z
        t.transform.rotation.w = msg.quat.w
        self.tf.sendTransform(t)

    def origGoalCb(self, msg, veh):
        self.voriggoal[veh] = msg

    def safeGoalCb(self, msg, veh):
        self.vsafegoal[veh] = msg

    def formationCb(self, msg):
        self.received_formation = True

    def assignmentCb(self, msg, veh):
        self.received_assignment = True

        if self.is_logging:
            self.log['assignments'][-1] += 1

    def statusCb(self, msg, veh):
        self.vstatus[veh] = msg

    #
    # State Machine
    #

    def tick(self):
        """State machine tick
        """

        # increment timer, used for waiting
        # n.b.: order matters since the first time each state runs this will
        # increment from -1 to 0
        self.timer_ticks += 1

        if self.state is State.WAITING_ON_FORMATION:
            if self.has_completed_trial():
                self.next_state(State.COMPLETE)
            elif self.has_received_formation() and self.has_elapsed(self.FORMATION_WAIT):
                self.clear_signals()
                self.received_formation = False
                self.received_assignment = False
                self.curr_formation_idx += 1
                self.next_state(State.WAITING_ON_ASSIGNMENT)

        elif self.state is State.WAITING_ON_ASSIGNMENT:
            if self.has_set_assignment() and self.has_started_flying():
                self.received_assignment = False
                self.start_logging()
                self.next_state(State.FLYING)

        elif self.state is State.FLYING:
            if self.has_converged():
                self.next_state(State.IN_FORMATION)
            elif self.has_gridlocked():
                self.next_state(State.GRIDLOCK)

        elif self.state is State.IN_FORMATION:
            if self.has_elapsed(self.CONVERGED_WAIT):
                self.stop_logging()
                self.next_state(State.WAITING_ON_FORMATION)
            elif not self.has_converged():
                self.next_state(State.FLYING)

        elif self.state is State.GRIDLOCK:
            if self.has_left_gridlock():
                self.next_state(State.FLYING)

        elif self.state is State.COMPLETE:
            self.complete()

        #
        # Log signals
        #

        if self.is_logging:
            self.log_signals()

    def next_state(self, state, reset=True):
        self.last_state = self.state
        self.state = state

        rospy.loginfo("Leaving {} for {}".
                        format(S(self.last_state), S(self.state)))

        # reset timer
        self.timer_ticks = -1

        # empty buffers
        if reset:
            self.buffers = {}

        #
        # Logging
        #

        # goint into GRIDLOCK
        if self.state is State.GRIDLOCK:
            # log time performing collision avoidance
            self.log['time_avoidance'][-1] = rospy.Time.now()

        # coming out of GRIDLOCK
        if self.last_state is State.GRIDLOCK:
            # update timing
            self.log['time_avoidance'][-1] = (rospy.Time.now()
                            - self.log['time_avoidance'][-1]).to_sec()

    #
    # Predicates
    #

    def has_elapsed(self, secs):
        elapsed = self.timer_ticks / self.tick_rate
        return elapsed >= secs

    def has_completed_trial(self):
        return self.curr_formation_idx >= self.NUM_FORMATIONS_IN_TRIAL

    def has_received_formation(self):
        return self.received_formation

    def has_set_assignment(self):
        return self.received_assignment

    def has_started_flying(self):
        c = [veh for (veh,msg) in self.voriggoal.items() if rospy.Time.now() - msg.header.stamp < rospy.Duration(0.1)]
        return len(c) == len(self.vehs)

    def has_converged(self):
        if 'converged_orig_vel' not in self.buffers:
            self.buffers['converged_orig_vel'] = deque(maxlen=self.BUFFLEN)

        # for convenience (note, deque is mutable---'by ref')
        buff_orig_vel = self.buffers['converged_orig_vel']

        # sample signals we need to determine predicate
        buff_orig_vel.append(self.sample_origgoal_speed_heading())

        # If we don't have enough data, we can't know the answer
        if len(buff_orig_vel) < self.BUFFLEN:
            return False

        # average each sample over vehicles
        avg_orig_mag, avg_orig_ang = np.array(buff_orig_vel).mean(axis=0).T

        # the swarm has converged to the desired formation
        # if the original motion planning goal is zero.
        return (avg_orig_mag < self.ORIG_ZERO_VEL_THR).all()

    def has_gridlocked(self):
        if 'gridlocked_active_ca' not in self.buffers:
            self.buffers['gridlocked_active_ca'] = deque(maxlen=self.BUFFLEN)

        # for convenience (note, deque is mutable---'by ref')
        buff_active_ca = self.buffers['gridlocked_active_ca']

        # sample signals we need to determine predicate
        buff_active_ca.append(self.sample_collision_avoidance_active())

        # If we don't have enough data, we can't know the answer
        if len(buff_active_ca) < self.BUFFLEN:
            return False

        # average each sample over vehicles
        avg_active_ca = np.array(buff_active_ca).mean(axis=0).T

        # the swarm is gridlocked if there exists a vehicle that is
        # on average in collision avoidance mode for too long
        return (avg_active_ca > self.AVG_ACTIVE_CA_THR).any()

    def has_left_gridlock(self):

        # check if we are in gridlock
        gridlocked = self.has_gridlocked()

        # If we don't have enough data, we can't know the answer
        if len(self.buffers['gridlocked_active_ca']) < self.BUFFLEN:
            return False

        return not gridlocked

    #
    # Actions
    #

    def clear_signals(self):
        self.voriggoal = {}
        self.vsafegoal = {}
        self.vstatus = {}

    def start_logging(self):
        if self.is_logging: return

        # initialize assignment counter
        if 'assignments' not in self.log:
            self.log['assignments'] = []

        # initialize time
        if 'time' not in self.log:
            self.log['time'] = []

        # initialize time in collision avoidance mode
        if 'time_avoidance' not in self.log:
            self.log['time_avoidance'] = []

        self.log['assignments'] += [1]
        self.log['time'] += [rospy.Time.now()]
        self.log['time_avoidance'] += [0]

        self.is_logging = True

    def stop_logging(self):
        if not self.is_logging: return
        self.is_logging = False

        # update timing
        self.log['time'][-1] = (rospy.Time.now()-self.log['time'][-1]).to_sec()
        rospy.loginfo("Convergence time: {:.2f}".format(self.log['time'][-1]))

    def complete(self):

        # # write logs to file
        # with open(self.datafile, 'a') as f:
        #     writer = csv.writer(f)
        #     writer.writerow([self.trial]
        #                         + self.log['dist'].tolist()
        #                         + self.log['time']
        #                         + self.log['time_avoidance']
        #                         + self.log['assignments'])

        return

    #
    # Helpers
    #

    def sample_origgoal_speed_heading(self):
        return [(np.linalg.norm((msg.vector.x, msg.vector.y, msg.vector.z)),
                    np.arctan2(msg.vector.y, msg.vector.x))
                for (veh,msg) in self.voriggoal.items()]

    def sample_safegoal_speed_heading(self):
        return [(np.linalg.norm((msg.vel.x, msg.vel.y, msg.vel.z)),
                    np.arctan2(msg.vel.y, msg.vel.x))
                for (veh,msg) in self.vsafegoal.items()]

    def sample_collision_avoidance_active(self):
        return [msg.collision_avoidance_active
                for (veh,msg) in self.vstatus.items()]

    def sample_position_x(self):
        return [msg.pose.position.x
                for (veh,msg) in self.vstates.items()]

    def sample_position_y(self):
        return [msg.pose.position.y
                for (veh,msg) in self.vstates.items()]

    def wrapToPi(self, x):
        return (x + np.pi) % (2 * np.pi) - np.pi

    def wrapTo2Pi(self, x):
        return x % (2 * np.pi)

    def log_signals(self):
        # sample the necessary signals
        x = np.array(self.sample_position_x())
        y = np.array(self.sample_position_y())

        # initialize filters
        if 'position_x' not in self.log:
            self.log['position_x'] = x
        if 'position_y' not in self.log:
            self.log['position_y'] = y
        if 'dist' not in self.log:
            self.log['dist'] = np.zeros_like(x)

        #
        # Signal Smoothing
        #

        lastx = self.log['position_x']
        self.log['position_x'] = self.alpha*lastx + (1-self.alpha)*x
        dx = np.abs(self.log['position_x'] - lastx)

        lasty = self.log['position_y']
        self.log['position_y'] = self.alpha*lasty + (1-self.alpha)*y
        dy = np.abs(self.log['position_y'] - lasty)

        # accumulate total planar distance traveled
        self.log['dist'] += np.sqrt(dx**2 + dy**2)


if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description='Supervise a simulation trial')
    # parser.add_argument('-n', '--name', type=str, help='filename to save data', default='aclswarm_trials')
    # parser.add_argument('-t', '--trial', type=int, help='trial number', required=True)
    # args = parser.parse_args()

    rospy.init_node('review_bag')
    node = Reviewer()
