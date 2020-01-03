#!/usr/bin/env python

from __future__ import division

import time
import rospy
import numpy as np

from behavior_selector.srv import MissionModeChange


class Supervisor:

    def __init__(self):

        # General swarm information
        self.vehs = rospy.get_param('/vehs')
        self.n = len(self.vehs)
        rospy.loginfo('Using vehicles: {}'.format(self.vehs))


if __name__ == '__main__':
    rospy.init_node('supervisor')
    node = Supervisor()
    rospy.spin()
