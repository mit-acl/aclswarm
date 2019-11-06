from __future__ import division

import time
import numpy as np
import cvxpy as cp


def createGainMatrix(adj, qDes):
    """
    Given an adjacency matrix and desired formation points, formulate and solve
    the corresponding semidefinite program (SDP) which yields the gain matrix A
    resulting in a stable distributed formation control.

    See K. Fathian et al., "Robust 3D Distributed Formation Control with 
    Collision Avoidance and Application to Multirotor Aerial Vehicles," ICRA'18

    Parameters
    ----------
    adj: nxn numpy array
        adjacency matrix encoding the sensing topology of swarm

    qDes: nx3 numpy array
        desired positions (in R^3) of all the agents w.r.t the current
        agent's coordinate frame.

    Returns
    -------

    A: 3nx3n numpy array
        symmetric nsd matrix with the last 5 or 6 (based on nullity) eigenvalues
        constrained to be zero. This gain matrix contains subblocks for each agent.
    """

    # Adjacency matrix must be set first
    if adj is None:
        return

    start_time = time.time()

    qs = qDes               # Desired formation coordinates in vector form
    n = np.shape(adj)[0]    # Number of agents

    oneX = np.kron(np.ones(n), [1, 0, 0])
    oneY = np.kron(np.ones(n), [0, 1, 0])
    oneZ = np.kron(np.ones(n), [0, 0, 1])

    # 90 deg rotation about z
    R = np.array([[0, -1, 0],    
                 [+1, 0, 0],    
                 [0, 0, +1]], dtype=np.float32)

    # Rotate 90 degrees about z-axis
    qsBar = np.zeros_like(qs)
    for i, vec in enumerate(qs):
        qsBar[i] = np.matmul(R, vec.T)

    # Project onto x-y plane
    qsp = np.copy(qs)
    qsp[:, 2] = 0

    # formations that have the same z coordinate are "flat", which causes
    # a rank deficiency in N. We use 'nullity' to choose the appropriate
    # number of linearly independent columns of U (from svd of N).
    flat = (np.std(qDes[:,2]) <= 1e-2)
    nullity = 5 if flat else 6

    # Kernel space
    N = np.vstack((qs.flatten(), qsBar.flatten(),
                   qsp.flatten(), oneX, oneY, oneZ)).T

    # Get orthogonal complement of kernel of A
    U, diag, Vh = np.linalg.svd(N)
    Q = U[:, nullity:3 * n]

    # Subspace constraints for given sensing topology
    S = 1 - np.kron(adj + np.eye(n), np.ones((3, 3)))

    # Solve via CVX
    A = cp.Variable((3 * n, 3 * n), symmetric=True)
    obj = cp.Maximize(cp.lambda_min(cp.matmul(Q.T, cp.matmul(A, Q))))
    constraints = [
        cp.matmul(A, N) == 0,    # Kernel of A
        cp.multiply(A, S) == 0,  # For agents that are not neighbors
        cp.norm(A) <= 10]

    for i in range(n):
        for j in range(n):
            if adj[i, j]:    # If agents are neighbors
                constraints.append(A[3 * i, 3 * j] ==
                                   A[3 * i + 1, 3 * j + 1])
                constraints.append(A[3 * i, 3 * j + 1]
                                   == -A[3 * i + 1, 3 * j])
                constraints.append(A[3 * i:3 * i + 2, 3 * j + 2] == 0)
                constraints.append(A[3 * i + 2, 3 * j:3 * j + 2] == 0)

    prob = cp.Problem(obj, constraints)
    prob.solve(eps=1e-6)

    end_time = time.time()
    print("Optimization completed in {:.3f} seconds".format(end_time - start_time))

    if prob.status not in ["infeasible", "unbounded"]:
        Ar = -np.copy(A.value)  # Make matrix negative semi-definite
        Ar /= np.max(np.abs(Ar))  # Scale matrix

        # Since we aren't very confident in our solver right now
        # (cvxpy seems to be choosing SCS to solve the SDP),
        # we will manually enforce symmetry of the gain matrix.
        Ar = 0.5*(np.array(Ar) + np.array(Ar).T)

        # Make sure that the last six eigenvalues are zero.
        # This means that N is the kernel of A.
        # We sort the eigenvalues so that the (supposedly) zero
        # eigenvalues can be found at the end of the list. We are
        # able to order the eigenvalues because the are guaranteed
        # to be real (symmetric A matrix)
        w, v = np.linalg.eig(Ar)
        w = np.sort(w)

        # if any eigenvalue is positive, error out
        if np.any(w>=1e-6):
            print
            print("ERROR: There is a positive eigenvalue in the calculated")
            print("gain matrix A.")
            print(w)
            print
            return -1

        # check that the last `nullity` eigenvalues are zero
        if np.linalg.norm(w[w.shape[0]-nullity:]) > 1e-6:
            print
            print("ERROR: Desired formation is not in the kernel of A.")
            print("The last {} eigenvalues of A should be zero.".format(nullity))
            print(w)
            print
            return -1

        # Make sure that the other eigenvalues are strictly negative.
        if np.any(np.real(w[:w.shape[0]-nullity])>=-1e-10):
            print
            print("ERROR: Eigenvalues corresponding to the formation")
            print("should be strictly negative, but are not.")
            print(w)
            print
            return -1

        print("Norm of last {} eigenvalues: {:.2e} (should be 0)".format(
                nullity, np.linalg.norm(w[w.shape[0]-nullity:])))
        print("Largest non-zero eigenvalue: {:.4f}".format(
                w[w.shape[0]-nullity-1]))

        return Ar
    else:
        return -1


if __name__ == '__main__':
    """Quick test for creating gain matrix"""
    
    # 4 agent line formation
    qdes = np.array([[0.0, 0.0, 0.0], 
                    [3.0, 0.0, 0.0],
                    [1.5, 0.0, 0.0],
                    [4.5, 0.0, 0.0]])

    # create fully-connected adjacency matrix
    n = qdes.shape[0]
    adj = np.ones((n,n)) - np.eye(n)

    A = createGainMatrix(adj, qdes)
    print(A)

