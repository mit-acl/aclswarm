#!/usr/bin/env python
"""
Generate Random Formation

This script will generate a formation group with K random 2D or 3D formations.
Note that formations cannot have agents on top of each other. In other words,
each agent is an infintely cylinder and no cylinders may intersect.
This is due to our collision avoidance strategy.
"""

from __future__ import division

import sys
import argparse

import rospy
import numpy as np; np.set_printoptions(linewidth=500)

def sample_point(l, w, h):
    x = np.random.uniform(low=-l/2.0, high=l/2.0)
    y = np.random.uniform(low=-w/2.0, high=w/2.0)
    z = np.random.uniform(low=0.0, high=h)
    return np.array([x, y, z])

def generate_formation(name, n, l, w, h, min_dist):

    # Two osculating cylinders have centroids min_dist away from each other.
    # The radii of each cylinder is min_dist / 2.
    r = min_dist / 2.0

    pts = np.array([])

    while np.atleast_2d(pts).shape[0] < n:
        overlapped = False

        # uniformly sample a point
        pt = sample_point(l, w, h)

        if np.atleast_2d(pts).shape[0] > 1:
            # check if this sampled cylinder collides with any other cylinder
            for p in np.atleast_2d(pts):
                d = np.linalg.norm(pt[0:2] - p[0:2])
                if d < 2 * r:
                    overlapped = True
                    break

        if overlapped is False:
            pts = pt if pts.shape[0] == 0 else np.vstack((pts, pt))

    return { 'name': name, 'points': pts.tolist() }


def generate_formation_group(n, fc, l, w, h, min_dist, k, graph):

    adjmat = np.ones((n,n), dtype=int) - np.eye(n, dtype=int)
    if not fc:
        # choose (at most) n-4 edges to remove (to maintain universal rigidity)
        m = np.random.randint(1, n - 4 + 1) # uniform in [1, n-4]

        # choose m random numbers in [0, n)
        rowIdx = np.random.choice(n, size=(m,))
        colIdx = np.random.choice(n, size=(m,))

        for i in range(m):
            adjmat[rowIdx[i], colIdx[i]] = 0
            adjmat[colIdx[i], rowIdx[i]] = 0

    formations = [
        generate_formation('A', n, l, w, h, min_dist),
        generate_formation('B', n, l, w, h, min_dist),
    ]

    if graph:
        import matplotlib.pyplot as plt
        import networkx as nx

        G = nx.from_numpy_matrix(adjmat)

        for i,f in enumerate(formations):
            plt.figure(i)
            pos = {j: pt[0:2] for j,pt in enumerate(f['points'])}
            nx.draw(G, pos=pos, with_labels=True)

    return {
        'agents': n,
        'adjmat': adjmat.tolist(),
        'formations': formations
    }

if __name__ == '__main__':

    parser = argparse.ArgumentParser(add_help=False, description="Generate a group of random formations")
    parser.add_argument('n', type=int, help="number of agents")
    parser.add_argument('-l', '--length', type=float, help="length of formation volume", default=10)
    parser.add_argument('-w', '--width', type=float, help="width of formation volume", default=10)
    parser.add_argument('-h', '--height', type=float, help="height of formation volume", default=10)
    parser.add_argument('-m', '--min-dist', type=float, help="centroid distances", default=1.5)
    parser.add_argument('-k', '--num-formations', type=int, help="num of formations", default=2)
    parser.add_argument('-fc', help="complete or randomly sparse graph", action="store_true")
    parser.add_argument('-g', '--graph', help="use NetworkX to graph formation", action="store_true")
    args = parser.parse_args()

    if args.n < 3:
        print("Not enough agents in the swarm!")
        sys.exit(1)

    # If there are less than five agents, the graph must be fully connected
    if not args.fc and args.n < 5:
        args.fc = True

    formgroup = generate_formation_group(args.n, args.fc, args.length,
                        args.width, args.height, args.min_dist,
                        args.num_formations, args.graph)
    rospy.set_param('/operator/simform', formgroup)

    if args.graph:
        import matplotlib.pyplot as plt
        plt.show()

