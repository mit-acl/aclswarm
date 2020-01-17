#!/usr/bin/env python

from __future__ import division

import time
import csv
from collections import deque

import rospy
import numpy as np; np.set_printoptions(linewidth=500)

from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Vector3Stamped
from aclswarm_msgs.msg import SafetyStatus
from behavior_selector.srv import MissionModeChange
from acl_msgs.msg import QuadGoal, ViconState

class State:
    IDLE = 1
    TAKING_OFF = 2
    HOVERING = 3
    WAITING_ON_ASSIGNMENT = 4
    FLYING = 5
    IN_FORMATION = 6
    GRIDLOCK = 7
    COMPLETE = 8
    TERMINATE = 9


def S(state):
    """Stringify state
    """
    if state == State.IDLE: return "IDLE"
    if state == State.TAKING_OFF: return "TAKING_OFF"
    if state == State.HOVERING: return "HOVERING"
    if state == State.WAITING_ON_ASSIGNMENT: return "WAITING_ON_ASSIGNMENT"
    if state == State.FLYING: return "FLYING"
    if state == State.IN_FORMATION: return "IN_FORMATION"
    if state == State.GRIDLOCK: return "GRIDLOCK"
    if state == State.COMPLETE: return "COMPLETE"
    if state == State.TERMINATE: return "TERMINATE"


class Supervisor:
    # for each predicate, how much data should be averaged over?
    BUFFER_SECONDS = 1

    # times for state machine---in units of seconds
    SIM_INIT_TIMEOUT = 10
    TAKE_OFF_TIMEOUT = 10
    HOVER_WAIT = 5
    CBAA_TIMEOUT = 10
    FORMATION_RECEIVED_WAIT = 1
    CONVERGED_WAIT = 1
    GRIDLOCK_TIMEOUT = 90

    # thresholds
    ZERO_POS_THR = 0.05 # m
    ORIG_ZERO_VEL_THR = 1.00 # m/s
    AVG_ACTIVE_CA_THR = 0.95 # percent

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')

        # formation information
        formation_group = rospy.get_param('/operator/formation_group')
        self.formations = rospy.get_param('/operator/{}'.
                                                format(formation_group))

        # flight information
        # since we are in sim and we always start with z=0, it
        # doesn't matter if the takeoff alt is relative or not
        self.takeoff_alt = rospy.get_param('/{}/safety/takeoff_alt'.
                                                format(self.vehs[0]))

        # for signal smoothing
        self.alpha =  0.98

        # ROS connections
        rospy.wait_for_service('change_mode')
        self.change_mode = rospy.ServiceProxy('change_mode', MissionModeChange)

        self.log = {}
        self.vstates = {}
        self.voriggoal = {}
        self.vsafegoal = {}
        self.vstatus = {}

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
            # status flags from Safety goal (i.e., collision avoidance active)
            rospy.Subscriber('/{}/safety/status'.format(veh), SafetyStatus,
                    lambda msg, v=veh: self.statusCb(msg, v), queue_size=1)

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

    def stateCb(self, msg, veh):
        self.vstates[veh] = msg

    def origGoalCb(self, msg, veh):
        self.voriggoal[veh] = msg

    def safeGoalCb(self, msg, veh):
        self.vsafegoal[veh] = msg

    def assignmentCb(self, msg, veh):
        self.received_assignment = True

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
                    self.next_state(State.COMPLETE)
                else:
                    self.next_formation()
                    self.next_state(State.WAITING_ON_ASSIGNMENT)

        elif self.state is State.WAITING_ON_ASSIGNMENT:
            if self.has_set_assignment():
                self.start_logging()
                self.next_state(State.FLYING)
            elif self.has_elapsed(self.CBAA_TIMEOUT):
                self.next_state(State.TERMINATE)

        elif self.state is State.FLYING:
            if self.has_elapsed(self.FORMATION_RECEIVED_WAIT):
                if self.has_converged():
                    self.next_state(State.IN_FORMATION, reset=False)
                elif self.has_gridlocked():
                    self.next_state(State.GRIDLOCK)

        elif self.state is State.IN_FORMATION:
            if self.has_elapsed(self.CONVERGED_WAIT):
                self.stop_logging()
                self.next_state(State.HOVERING)
            elif not self.has_converged():
                self.next_state(State.FLYING)

        elif self.state is State.GRIDLOCK:
            if self.has_left_gridlock():
                self.next_state(State.FLYING)
            elif self.has_elapsed(self.GRIDLOCK_TIMEOUT):
                self.next_state(State.TERMINATE)

        elif self.state is State.COMPLETE:
            self.complete()

        elif self.state is State.TERMINATE:
            self.terminate()

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

    def start_logging(self):
        if self.is_logging: return
        self.is_logging = True

        # initialize time
        if 'time' not in self.log:
            self.log['time'] = []

        self.log['time'] += [rospy.Time.now()]

    def stop_logging(self):
        if not self.is_logging: return
        self.is_logging = False

        # update timing
        self.log['time'][-1] = (rospy.Time.now()-self.log['time'][-1]).to_sec()
        rospy.loginfo("Convergence time: {:.2f}".format(self.log['time'][-1]))

    def complete(self):

        # write logs to file
        with open(r'aclswarm_trials.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow(self.log['dist'].tolist() + self.log['time'])

        rospy.signal_shutdown("trial completed successfully")

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
    rospy.init_node('supervisor')
    node = Supervisor()
