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

    _, sQ, _ = np.linalg.svd(q.T - q.T.mean(0))
    _, sP, _ = np.linalg.svd(p.T - p.T.mean(0))

    # determine the rank of the formation
    RANK_TH = 0.05 # be very stringent
    d = 3 if np.all(sQ > RANK_TH*sQ[0]) and np.all(sP > RANK_TH*sP[0]) else 2

    T = arun(q[0:d,:], p[0:d,:])

    # make sure to embed in 3D if necessary
    if d == 2:
        R = np.eye(3)
        R[0:2,0:2] = T[0]
        t = np.zeros((3,))
        t[0:2] = T[1]
    else:
        R = T[0]
        t = [1]

    paligned = np.dot(R, p) + np.tile(t, (p.shape[1], 1)).T

    return paligned

def find_optimal_assignment(q, p):
    """
    Finds the optimal assignment between swarm vehicles and the desired formation

    in:
      q   dxn np.array (d is 2D or 3D) - current state
      p   dxn np.array                 - desired formation
    out:
      P   length-n list, permutation vector that maps vehid to formpt
    """

    # align the desired formation to the swarm
    paligned = align(q, p)

    # construct pairwise-distance matrix to use as cost
    # n.b.: vehid changes per row, so col_ind tells us which task for vehid
    S = cdist(q.T, paligned.T)

    # find the optimal assignment via Hungarian
    # n.b.: P is a rowvec that maps vehid to formpt
    _, P = linear_sum_assignment(S)

    return P.tolist()

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


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # initialized swarm (in grid)
    q = np.array([[-4.0, 0.0, 1.0],
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


    # test_arun(q)
    # test_align(q, p)
    # test_assign(q, p)

    pa = align(q, p)
    assign = find_optimal_assignment(q, p)
    print(assign)

    MATLAB_blue = [0, 0.4470, 0.7410]
    MATLAB_red = [0.8500, 0.3250, 0.0980]

    n = q.shape[1]

    #
    # Plot the alignment
    #

    plt.figure(1)
    plt.title('Without Assignment')
    plt.grid(alpha=0.3)
    plt.axis('equal')

    # plot current swarm positions
    plt.scatter(q[0,:], q[1,:], s=350, c=MATLAB_blue)
    for i in range(n):
        plt.annotate(str(i+1), xy=(q[0,i], q[1,i]), ha='center', va='center', fontsize=11, weight="bold")

    # plot aligned desired formation
    plt.scatter(pa[0,:], pa[1,:], s=350, c=MATLAB_red)
    for i in range(n):
        plt.annotate(str(i+1), xy=(pa[0,i], pa[1,i]), ha='center', va='center', fontsize=11, weight="bold")

    # plot assignment lines
    for i in range(n):
        plt.plot([q[0,i], pa[0,i]], [q[1,i], pa[1,i]], c=(0,0,0,0.5))

    #
    # Plot the assignment
    #

    plt.figure(2)
    plt.title('With Assignment')
    plt.grid(alpha=0.3)
    plt.axis('equal')

    # apply the assignment
    # n.b., the assignment maps vehid to formpt; however, when used to index
    # into a list, it becomes the inverse perm. Thus, we bring the formpts
    # into order with the vehicles.
    pa = pa[:,assign]

    # plot current swarm positions
    plt.scatter(q[0,:], q[1,:], s=350, c=MATLAB_blue)
    for i in range(n):
        plt.annotate(str(i+1), xy=(q[0,i], q[1,i]), ha='center', va='center', fontsize=11, weight="bold")

    # plot aligned desired formation
    plt.scatter(pa[0,:], pa[1,:], s=350, c=MATLAB_red)
    for i in range(n):
        plt.annotate(str(i+1), xy=(pa[0,i], pa[1,i]), ha='center', va='center', fontsize=11, weight="bold")

    # plot assignment lines
    for i in range(n):
        plt.plot([q[0,i], pa[0,i]], [q[1,i], pa[1,i]], c=(0,0,0,0.5))

    plt.show()
