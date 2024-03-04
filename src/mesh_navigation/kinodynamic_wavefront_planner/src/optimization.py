#!/usr/bin/env python3
import numpy as np
import cvxpy as cp
import sys

def optimize(u, v, l, theta_max):
    d = cp.Variable(len(u))
    objective = cp.Maximize(cp.matmul(d.T, u))
    constraints = [cp.matmul(v.T, d) >= cp.norm(v) * np.cos(theta_max),
                   cp.norm(d) <= 1]
    prob = cp.Problem(objective, constraints)
    prob.solve()

    return d.value

if __name__ == "__main__":
    u = np.fromstring(sys.argv[1], sep=',')
    v = np.fromstring(sys.argv[2], sep=',')
    theta_max =  float(sys.argv[3])

    optimal_d = optimize(u, v, 1, theta_max)
    print(optimal_d)
