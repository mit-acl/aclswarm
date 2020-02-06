"""
This assignment code is only used for comparing the distributed assignment
of ACLswarm to a centralized Hungarian method. When using a centralized
coordinator, we have access to global information (i.e,. the state of the
entire swarm). This allows us to optimally assign each vehicle in the swarm
to a formation point (i.e., no crossings). However, it requires O(n^3)
computation and is not as scalable as using the method proposed in ACLswarm.
"""

import numpy as np; np.set_printoptions(linewidth=500)

from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

def arun(q, p):
    """Arun's Method

    Minimizes ||q - (Rp + t)||^2

    in:
        q   dxn np.array (d is 2D or 3D)
        p   dxn np.array
    out:
        (R, t)  SE(d) rigid body transformation
    """
    assert q.shape[0] == p.shape[0]
    assert q.shape[1] == p.shape[1]
    assert q.shape[1] > 0

    d, n = q.shape

    # shift point clouds by centroids
    mu_q = q.mean(1)
    mu_p = p.mean(1)
    Q = q - np.tile(mu_q, (n, 1)).T
    P = p - np.tile(mu_p, (n, 1)).T


    # construct H matrix (dxd)
    H = np.matmul(Q, P.T)

    # perform SVD of H
    U, _, Vt = np.linalg.svd(H, full_matrices=True, compute_uv=True)
    D = np.eye(d)
    D[d-1,d-1] = np.linalg.det(np.matmul(U, Vt))

    # solve rotation-only problem
    R = np.matmul(np.matmul(U, D), Vt)

    # solve translation
    t = mu_q - np.matmul(R, mu_p)

    return R, t

def align(q, p):
    """Align

    Aligns the desired formation p onto the current swarm state q.
    This is a wrapper to decide if we should do 2D or 3D Arun.

    in:
        q   dxn np.array (d is 2D or 3D) - current state
        p   dxn np.array                 - desired formation
    out:
        paligned    dxn np.array         - (R*p + t)
    """

    # _, sQ, _ = np.linalg.svd(q.T - q.T.mean(0))
    # _, sP, _ = np.linalg.svd(p.T - p.T.mean(0))

    # # determine the rank of the formation
    # RANK_TH = 0.05 # be very stringent
    # d = 3 if np.all(sQ > RANK_TH*sQ[0]) and np.all(sP > RANK_TH*sP[0]) else 2

    # our control is only invariant to rot about z -- always use 2D umeyama
    d = 2

    T = arun(q[0:d,:], p[0:d,:])

    # make sure to embed in 3D if necessary
    if d == 2:
        R = np.eye(3)
        R[0:2,0:2] = T[0]
        t = np.zeros((3,))
        t[0:2] = T[1]
    else:
        R = T[0]
        t = T[1]

    paligned = np.dot(R, p) + np.tile(t, (p.shape[1], 1)).T

    return paligned

def find_optimal_assignment(q, p, last=None):
    """
    Finds the optimal assignment between swarm vehicles and the desired formation

    in:
      q     dxn np.array (d is 2D or 3D) - current state
      p     dxn np.array                 - desired formation
      last  length-n list, last assignment (perm vector maps vehid to formpt).
            If None, assume identity map.
    out:
      P   length-n list, permutation vector that maps vehid to formpt
    """

    #
    # Alignment (uses last assignment)
    #

    # create identity map if there was no last assignment
    if last is None:
        last = [i for i in range(q.shape[1])]

    # create a vector so that indices are aligned with assignment
    qq = np.zeros_like(q)
    for (vehid,formpt) in enumerate(last):
        qq[:,formpt] = q[:,vehid]

    # align the desired formation to the swarm (using current assignment)
    paligned = align(qq, p)

    #
    # Assignment (maps original vehid to formpt)
    #

    # construct pairwise-distance matrix to use as cost
    # n.b.: vehid changes per row, so col_ind tells us which task for vehid
    # n.b.: we are creating a cost btwn vehicles and formpts; this will NOT
    #       create an incremental permutation.
    S = cdist(q.T, paligned.T)

    # find the optimal assignment via Hungarian
    # n.b.: P is a rowvec that maps vehid to formpt
    _, P = linear_sum_assignment(S)

    return P.tolist(), paligned

# -----------------------------------------------------------------------------
# Test Code
# -----------------------------------------------------------------------------

def test_arun(q):
    R = np.array([[0.9689135, -0.0232753, 0.2463025],
                  [0.0236362, 0.9997195, 0.0014915],
                  [-0.2462682, 0.0043765, 0.9691918]])
    t = np.array([[1], [2], [3]])
    p = np.matmul(R.T, q-t)
    
    print(arun(q, p))

def test_align(q, p):
    print(align(q, p))

def test_assign(q, p):
    print(find_optimal_assignment(q, p))

def plot_swarm(q, pa, P=None):
    # how many agents in swarm
    n = q0.shape[1]

    MATLAB_blue = lambda alpha=1: np.array([[0, 0.4470, 0.7410, alpha]])
    MATLAB_red = lambda alpha=1: np.array([[0.8500, 0.3250, 0.0980, alpha]])

    # plot current swarm positions (as letters)
    plt.scatter(q[0,:], q[1,:], s=350, c=MATLAB_blue())
    for i in range(n):
        plt.annotate(chr(65+i), xy=(q[0,i], q[1,i]), ha='center', va='center', fontsize=11, weight="bold")

    # plot aligned desired formation (as numbers)
    plt.scatter(pa[0,:], pa[1,:], s=350, c=MATLAB_red(0.6))
    for i in range(n):
        plt.annotate(str(i+1), xy=(pa[0,i], pa[1,i]), ha='center', va='center', fontsize=11, weight="bold")

    if P is not None:
        # Apply the assignment
        # n.b., the assignment maps vehid to formpt; however, when used to index
        # into a list, it becomes the inverse perm. Thus, we bring the formpts
        # into order with the vehicles.
        paa = pa[:,P]
    else:
        paa = pa

    # plot assignment lines
    for i in range(n):
        plt.plot([q[0,i], paa[0,i]], [q[1,i], paa[1,i]], c=(0,0,0,0.5))


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # initialized swarm (in grid)
    q0 = np.array([[-4.0, 0.0, 1.0],
                   [-2.5, 0.0, 1.0],
                   [-1.0, 0.0, 1.0],
                   [ 0.5, 0.0, 1.0],
                   [ 2.0, 0.0, 1.0],
                   [-4.0, 1.5, 1.0],
                   [-2.5, 1.5, 1.0],
                   [-1.0, 1.5, 1.0],
                   [ 0.5, 1.5, 1.0],
                   [ 2.0, 1.5, 1.0],
                   [-4.0, 3.0, 1.0],
                   [-2.5, 3.0, 1.0],
                   [-1.0, 3.0, 1.0],
                   [ 0.5, 3.0, 1.0],
                   [ 2.0, 3.0, 1.0]]).T

    # next swarm state
    q1 = np.array([[ 0.8849, -4.5246, 0.9196],
                   [-2.9238,  1.1226, 0.9069],
                   [ 0.7317,  3.0540, 0.8859],
                   [-3.1139,  4.7813, 0.9014],
                   [-2.9776, -0.7493, 0.9173],
                   [ 0.8363, -0.6425, 0.8929],
                   [-2.7897, -4.5020, 0.9105],
                   [ 0.7396,  0.9338, 0.9106],
                   [-2.8721, -2.7825, 0.9060],
                   [-0.9893, -2.5827, 0.9231],
                   [-0.9864, -4.1532, 0.8873],
                   [-1.1011, -0.6975, 0.9154],
                   [-3.0136,  3.0430, 0.9114],
                   [-1.1074,  2.7814, 0.8955],
                   [-0.9901,  1.1900, 0.8918]]).T

    # next swarm state
    q2 = np.array([[ 0.7228, -3.7401, 0.9202],
                   [-3.5939,  1.6082, 0.9045],
                   [ 0.2830,  2.3955, 0.8884],
                   [-3.7610,  4.2075, 0.9120],
                   [-3.1410, -0.2429, 0.9148],
                   [ 1.3119, -1.2218, 0.8894],
                   [-2.3583, -3.9815, 0.9110],
                   [ 0.7413,  0.8380, 0.9025],
                   [-2.6446, -1.9076, 0.9082],
                   [-0.2375, -2.3977, 0.9257],
                   [-0.8326, -4.7105, 0.8902],
                   [-0.9370, -0.6721, 0.9142],
                   [-2.3532,  2.4065, 0.9095],
                   [-0.8575,  3.4629, 0.9034],
                   [-1.7998,  0.9387, 0.8981]]).T

    # next swarm state
    q3 = np.array([[ 0.6977, -4.2559, 0.9208],
                   [-3.0213,  1.2084, 0.9104],
                   [ 0.6140,  2.8713, 0.8898],
                   [-3.2891,  4.6929, 0.9098],
                   [-2.9772, -0.6642, 0.9190],
                   [ 0.9119, -0.7935, 0.8918],
                   [-2.8531, -4.3338, 0.9123],
                   [ 0.7289,  1.0869, 0.9086],
                   [-2.8635, -2.2500, 0.9099],
                   [-1.1273, -2.5051, 0.9252],
                   [-1.0824, -4.3189, 0.8820],
                   [-1.1120, -0.6522, 0.9098],
                   [-2.9075,  3.0017, 0.9131],
                   [-1.0886,  3.1643, 0.9062],
                   [-1.2231,  1.2266, 0.8983]]).T

       # desired formation - MIT
    p = np.array([[0.0, 0.0, 0.0],
                  [0.0, 1.5, 0.0],
                  [0.0, 3.0, 0.0],
                  [1.5, 1.5, 0.0],
                  [3.0, 1.5, 0.0],
                  [1.5, 3.0, 0.0],
                  [4.5, 1.5, 0.0],
                  [3.0, 0.0, 0.0],
                  [3.0, 3.0, 0.0],
                  [4.5, 0.0, 0.0],
                  [4.5, 3.0, 0.0],
                  [6.0, 1.5, 0.0],
                  [7.5, 3.0, 0.0],
                  [6.0, 3.0, 0.0],
                  [6.0, 0.0, 0.0]]).T


    # test_arun(q0)
    # test_align(q0, p)
    # test_assign(q0, p)

    #
    # Plot the swarm, just after new formation, without assignment
    #

    pa0 = align(q0, p)

    plt.figure(1)
    plt.title('Without Assignment')
    plt.grid(alpha=0.3)
    plt.axis('equal')
    plot_swarm(q0, pa0)

    #
    # Plot the swarm, just after new formation, with assignment
    #

    P0, pa0 = find_optimal_assignment(q0, p, last=None)
    print(P0)

    plt.figure(2)
    plt.title('With Assignment (0)')
    plt.grid(alpha=0.3)
    plt.axis('equal')
    plot_swarm(q0, pa0, P0)

    # -------------------------------------------------------------------------

    #
    # Plot the swarm, on its way into formation, with new assignment
    #

    # Note: the following three are from an experiment separate from
    # the previous two. This is why last=None.

    P1, pa1 = find_optimal_assignment(q1, p, last=None)
    print(P1)

    plt.figure(3)
    plt.title('With Assignment (1)')
    plt.grid(alpha=0.3)
    plt.axis('equal')
    plot_swarm(q1, pa1, P1)

    #
    # Plot the swarm, on its way into formation, with new assignment
    #

    P2, pa2 = find_optimal_assignment(q2, p, last=P1)
    print(P2)

    plt.figure(4)
    plt.title('With Assignment (2)')
    plt.grid(alpha=0.3)
    plt.axis('equal')
    plot_swarm(q2, pa2, P2)

    #
    # Plot the swarm, on its way into formation, with new assignment
    #

    P3, pa3 = find_optimal_assignment(q3, p, last=P2)
    print(P3)

    plt.figure(5)
    plt.title('With Assignment (3)')
    plt.grid(alpha=0.3)
    plt.axis('equal')
    plot_swarm(q3, pa3, P3)

    plt.show()
