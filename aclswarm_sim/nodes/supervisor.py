#!/usr/bin/env python

from __future__ import division

import time
from collections import deque

import rospy
import numpy as np; np.set_printoptions(linewidth=500)

from behavior_selector.srv import MissionModeChange
from acl_msgs.msg import QuadGoal, ViconState
from geometry_msgs.msg import Vector3Stamped

class State:
    IDLE = 1
    TAKING_OFF = 2
    HOVERING = 3
    FLYING = 4
    GRIDLOCK = 5
    TERMINATE = 6


def S(state):
    """Stringify state
    """
    if state == State.IDLE: return "IDLE"
    if state == State.TAKING_OFF: return "TAKING_OFF"
    if state == State.HOVERING: return "HOVERING"
    if state == State.FLYING: return "FLYING"
    if state == State.GRIDLOCK: return "GRIDLOCK"
    if state == State.TERMINATE: return "TERMINATE"


class Supervisor:
    # for each predicate, how much data should be averaged over?
    BUFFER_SECONDS = 1

    # times for state machine---in units of seconds
    SIM_INIT_TIMEOUT = 10
    TAKE_OFF_TIMEOUT = 5
    HOVER_WAIT = 5
    FORMATION_RECEIVED_WAIT = 5
    GRIDLOCK_TIMEOUT = 60

    # thresholds
    ZERO_POS_THR = 0.05 # m
    ORIG_VEL_THR = 1.00 # m/s
    SAFE_VEL_THR = 0.15 # m/s

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        rospy.loginfo('Using vehicles: {}'.format(self.vehs))

        # formation information
        formation_group = rospy.get_param('/operator/formation_group')
        self.formations = rospy.get_param('/operator/{}'.
                                                format(formation_group))

        # flight information
        # since we are in sim and we always start with z=0, it
        # doesn't matter if the takeoff alt is relative or not
        self.takeoff_alt = rospy.get_param('/{}/safety/takeoff_alt'.
                                                format(self.vehs[0]))

        # ROS connections
        rospy.wait_for_service('change_mode')
        self.change_mode = rospy.ServiceProxy('change_mode', MissionModeChange)

        self.vstates = {}
        self.voriggoal = {}
        self.vsafegoal = {}

        for veh in self.vehs:
            # ground truth state of each vehicle
            rospy.Subscriber('/{}/vicon'.format(veh), ViconState,
                    lambda msg, v=veh: self.stateCb(msg, v), queue_size=1)
            # the desired velocity goal from the distributed motion planner
            rospy.Subscriber('/{}/distcmd'.format(veh), Vector3Stamped,
                    lambda msg, v=veh: self.origGoalCb(msg, v), queue_size=1)
            # the safe, collision-free version of the motion planner vel goal
            rospy.Subscriber('/{}/goal'.format(veh), QuadGoal,
                    lambda msg, v=veh: self.safeGoalCb(msg, v), queue_size=1)

        # initialize state machine variables
        self.state = State.IDLE
        self.last_state = None
        self.timer_ticks = -1
        self.curr_formation_idx = -1
        self.tick_rate = 50

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

    def stateCb(self, msg, veh):
        self.vstates[veh] = msg

    def origGoalCb(self, msg, veh):
        self.voriggoal[veh] = msg

    def safeGoalCb(self, msg, veh):
        self.vsafegoal[veh] = msg

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

        if self.state is State.IDLE:
            if self.has_sim_initialized():
                self.takeoff()
                self.next_state(State.TAKING_OFF)
            elif self.has_elapsed(self.SIM_INIT_TIMEOUT):
                self.next_state(State.TERMINATE)

        elif self.state is State.TAKING_OFF:
            if self.has_taken_off():
                self.next_state(State.HOVERING)
            elif self.has_elapsed(self.TAKE_OFF_TIMEOUT):
                self.next_state(State.TERMINATE)

        elif self.state is State.HOVERING:
            if self.has_elapsed(self.HOVER_WAIT):
                if self.has_cycled_through_formations():
                    self.next_state(State.TERMINATE)
                else:
                    self.next_formation()
                    self.next_state(State.FLYING)

        elif self.state is State.FLYING:
            if self.has_elapsed(self.FORMATION_RECEIVED_WAIT):
                if self.has_converged():
                    self.next_state(State.HOVERING)
                elif self.has_gridlocked():
                    self.next_state(State.GRIDLOCK)

        elif self.state is State.GRIDLOCK:
            if self.has_left_gridlock():
                self.next_state(State.FLYING)
            elif self.has_elapsed(self.GRIDLOCK_TIMEOUT):
                self.next_state(State.TERMINATE)

        elif self.state is State.TERMINATE:
            self.terminate()

    def next_state(self, state):
        self.last_state = self.state
        self.state = state

        rospy.loginfo("Leaving {} for {}".
                        format(S(self.last_state), S(self.state)))

        # reset timer
        self.timer_ticks = -1

        # empty buffers
        self.buffers = {}

    #
    # Predicates
    #

    def has_elapsed(self, secs):
        elapsed = self.timer_ticks / self.tick_rate
        return elapsed >= secs

    def has_sim_initialized(self):
        # the simulation is ready if we have received states
        # for each of the vehicles in the swarm
        return len(self.vstates.keys()) == len(self.vehs)

    def has_taken_off(self):
        z = [msg.pose.position.z for (veh,msg) in self.vstates.items()]

        # the swarm has taken off if every vehicle has achieved
        # the takeoff altitude
        return (np.abs(np.array(z) - self.takeoff_alt)
                                < self.ZERO_POS_THR).all()

    def has_converged(self, sample=True):
        if 'norm_vel_origgoal' not in self.buffers:
            self.buffers['norm_vel_origgoal'] = deque(maxlen=self.BUFFLEN)

        if sample:
            v = [np.linalg.norm((msg.vector.x, msg.vector.y, msg.vector.z))
                    for (veh,msg) in self.voriggoal.items()]

            self.buffers['norm_vel_origgoal'].append(v)

        # If we don't have enough data, we can't know the answer
        if len(self.buffers['norm_vel_origgoal']) < self.BUFFLEN:
            return False

        # average each sample over vehicles
        vbuff = np.array(self.buffers['norm_vel_origgoal']).mean(axis=0)

        # the swarm has converged to the desired formation
        # if the original motion planning goal is zero.
        return (vbuff < self.ORIG_VEL_THR).all()

    def has_gridlocked(self, sample=True):
        if 'norm_vel_safegoal' not in self.buffers:
            self.buffers['norm_vel_safegoal'] = deque(maxlen=self.BUFFLEN)

        if sample:
            v = [np.linalg.norm((msg.vel.x, msg.vel.y, msg.vel.z))
                    for (veh,msg) in self.vsafegoal.items()]

            self.buffers['norm_vel_safegoal'].append(v)

        # If we don't have enough data, we can't know the answer
        if len(self.buffers['norm_vel_safegoal']) < self.BUFFLEN:
            return False

        # average each sample over vehicles
        vbuff = np.array(self.buffers['norm_vel_safegoal']).mean(axis=0)

        # the swarm is gridlocked if we haven't converged
        # but the safe velocity commands are zero.
        return (not self.has_converged(sample=False) and
                    (vbuff < self.SAFE_VEL_THR).all())

    def has_left_gridlock(self, sample=True):

        # check if we are in gridlock
        gridlocked = self.has_gridlocked(sample)

        # If we don't have enough data, we can't know the answer
        if len(self.buffers['norm_vel_safegoal']) < self.BUFFLEN:
            return False

        return not gridlocked

    def has_cycled_through_formations(self):
        # We want to cycle through formations and end up converged to first one
        return self.curr_formation_idx == len(self.formations['formations'])

    #
    # Actions
    #

    def takeoff(self):
        self.change_mode(MissionModeChange._request_class.START)

    def next_formation(self):
        self.curr_formation_idx += 1

        idx = self.curr_formation_idx % len(self.formations['formations'])
        rospy.logwarn("Current formation: {}".
                            format(self.formations['formations'][idx]['name']))

        # request operator to send next formation
        self.change_mode(MissionModeChange._request_class.START)

    def terminate(self):
        rospy.signal_shutdown("from state {}".format(S(self.last_state)))


if __name__ == '__main__':
    rospy.init_node('supervisor')
    node = Supervisor()
