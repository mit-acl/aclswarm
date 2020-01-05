#!/usr/bin/env python

from __future__ import division

import time
from collections import deque

import rospy
import numpy as np; np.set_printoptions(linewidth=500)

from behavior_selector.srv import MissionModeChange
from acl_msgs.msg import QuadGoal, ViconState
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Vector3Stamped

class State:
    IDLE = 1
    TAKING_OFF = 2
    HOVERING = 3
    WAITING_ON_ASSIGNMENT = 4
    FLYING = 5
    GRIDLOCK = 6
    TERMINATE = 7


def S(state):
    """Stringify state
    """
    if state == State.IDLE: return "IDLE"
    if state == State.TAKING_OFF: return "TAKING_OFF"
    if state == State.HOVERING: return "HOVERING"
    if state == State.WAITING_ON_ASSIGNMENT: return "WAITING_ON_ASSIGNMENT"
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
    CBAA_TIMEOUT = 10
    FORMATION_RECEIVED_WAIT = 5
    GRIDLOCK_TIMEOUT = 60

    # thresholds
    ZERO_POS_THR = 0.05 # m
    ORIG_ZERO_VEL_THR = 1.00 # m/s
    SAFE_ZERO_VEL_THR = 0.15 # m/s
    ANG_DIFF_THR = np.pi/60.0 # 3 degrees

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

        for idx, veh in enumerate(self.vehs):
            # ground truth state of each vehicle
            rospy.Subscriber('/{}/vicon'.format(veh), ViconState,
                    lambda msg, v=veh: self.stateCb(msg, v), queue_size=1)
            # the desired velocity goal from the distributed motion planner
            rospy.Subscriber('/{}/distcmd'.format(veh), Vector3Stamped,
                    lambda msg, v=veh: self.origGoalCb(msg, v), queue_size=1)
            # the safe, collision-free version of the motion planner vel goal
            rospy.Subscriber('/{}/goal'.format(veh), QuadGoal,
                    lambda msg, v=veh: self.safeGoalCb(msg, v), queue_size=1)

            # we only need one subscriber for the following
            if idx == 0:
                # an assignment was generated
                rospy.Subscriber('/{}/assignment'.format(veh), UInt8MultiArray,
                    lambda msg, v=veh: self.assignmentCb(msg, v), queue_size=1)

        # initialize state machine variables
        self.state = State.IDLE
        self.last_state = None
        self.timer_ticks = -1
        self.curr_formation_idx = -1
        self.received_assignment = False
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

    def assignmentCb(self, msg, veh):
        self.received_assignment = True

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
                    self.next_state(State.WAITING_ON_ASSIGNMENT)

        elif self.state is State.WAITING_ON_ASSIGNMENT:
            if self.has_set_assignment():
                self.next_state(State.FLYING)
            elif self.has_elapsed(self.CBAA_TIMEOUT):
                self.next_state(State.TERMINATE)

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

    def has_set_assignment(self):
        return self.received_assignment

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
        if 'gridlocked_orig_vel' not in self.buffers:
            self.buffers['gridlocked_orig_vel'] = deque(maxlen=self.BUFFLEN)
        if 'gridlocked_safe_vel' not in self.buffers:
            self.buffers['gridlocked_safe_vel'] = deque(maxlen=self.BUFFLEN)

        # for convenience (note, deque is mutable---'by ref')
        buff_orig_vel = self.buffers['gridlocked_orig_vel']
        buff_safe_vel = self.buffers['gridlocked_safe_vel']

        # sample signals we need to determine predicate
        buff_orig_vel.append(self.sample_origgoal_speed_heading())
        buff_safe_vel.append(self.sample_safegoal_speed_heading())

        # If we don't have enough data, we can't know the answer
        if len(buff_orig_vel) < self.BUFFLEN:
            return False

        # average each sample over vehicles
        avg_orig_mag, avg_orig_ang = np.array(buff_orig_vel).mean(axis=0).T
        avg_safe_mag, avg_safe_ang = np.array(buff_safe_vel).mean(axis=0).T

        # a vehicle is safety stopped when its safe speed is zero but its
        # desired speed from motion planning in nonzero
        safety_stop = np.logical_and(avg_safe_mag < self.SAFE_ZERO_VEL_THR,
                                        avg_orig_mag > self.SAFE_ZERO_VEL_THR)

        # a vehicle is safety avoiding when its safe speed direction is
        # different than the motion planning speed direction
        ang_diff = np.abs(self.wrapToPi(self.wrapToPi(avg_orig_ang)
                                            - self.wrapToPi(avg_safe_ang)))
        safety_avoid = (ang_diff > self.ANG_DIFF_THR)

        # a vehicle is in collision avoidance mode if it is safety stopped
        # or if is safety avoiding
        # collision_avoidance = np.logical_or(safety_stop, safety_avoid)
        collision_avoidance = safety_stop

        # the swarm is gridlocked if there exists a vehicle that is
        # in collision avoidance mode
        return collision_avoidance.any()

    def has_left_gridlock(self):

        # check if we are in gridlock
        gridlocked = self.has_gridlocked()

        # If we don't have enough data, we can't know the answer
        if len(self.buffers['gridlocked_orig_vel']) < self.BUFFLEN:
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

        # we expect every formation to generate an assignment.
        # clear the last assignment flag so we can wait for the next one
        self.received_assignment = False

        # request operator to send next formation
        self.change_mode(MissionModeChange._request_class.START)

    def terminate(self):
        rospy.signal_shutdown("from state {}".format(S(self.last_state)))

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

    def wrapToPi(self, x):
        return (x + np.pi) % (2 * np.pi) - np.pi

    def wrapTo2Pi(self, x):
        return x % (2 * np.pi)



if __name__ == '__main__':
    rospy.init_node('supervisor')
    node = Supervisor()
