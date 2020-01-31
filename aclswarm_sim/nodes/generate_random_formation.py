#!/usr/bin/env python

from __future__ import division

import argparse

import rospy
import numpy as np; np.set_printoptions(linewidth=500)


def generate_formation(name, n, l, w, h, min_dist):

    formation = []

    return { 'name': name, 'points': formation }


def generate_formation_group(n, fc, l, w, h, min_dist):

    if fc:
        adjmat = 'fc'
    else:
        adjmat = np.ones((n,n)) - np.eye(n)

    formations = [
        generate_formation('A', n, l, w, h, min_dist),
        generate_formation('B', n, l, w, h, min_dist),
    ]

    return {
        'agents': n,
        'adjmat': adjmat,
        'formations': formations
    }

if __name__ == '__main__':

    n = 5
    fc = True
    l = 10
    w = 10
    h = 3
    min_dist = 0.75

    formgroup = generate_formation_group(n, fc, l, w, h, min_dist)
    rospy.set_param('/operator/simform', formgroup)
