from __future__ import division
import numpy as np; npl = np.linalg
from scipy.optimize import minimize

labels = ["FLV"   , "FRV"   , "BLV"   , "BRV"   , "FLL"  , "FRL"  , "BLL"  , "BRL" ]
B8 = np.array([[ 0.0000,  0.0000,  0.0   ,  0.0   ,  0.866, 0.866, -0.866, -0.866],
               [ 0.0000,  0.0000,  0.0   ,  0.0   , -0.5  , 0.500, -0.5  ,  0.5  ],
               [ 1.0000,  1.0000,  1.0   ,  1.0   ,  0.0  , 0.000,  0.0  ,  0.0  ],
               [ 0.169 , -0.169 ,  0.1690, -0.169 ,  0.0  , 0.000,  0.0  , -0.0  ],
               [-0.1583, -0.1583,  0.1583,  0.1583,  0.0  , 0.000,  0.0  ,  0.0  ],
               [ 0.0   ,  0.0   ,  0.0   ,  0.0   , -0.376, 0.376,  0.376, -0.376]])

B7 = np.copy(B8)
B7[:, 0] = 0

B6 = np.copy(B7)
B6[:, 2] = 0

B = B8

min_thrusts = np.array([-75, -75, -75, -75, -75, -75, -75, -75])
max_thrusts = np.array([105, 105, 105, 105, 105, 105, 105, 105])

thrust_cost = np.diag([0.1] * 8)

wrench = np.array([0, 0, 0, 0, 0, 100])


def compute(wrench, thrust_weight):
    thrust_cost = np.diag([thrust_weight] * 8)

    def objective(u):
        error_cost = np.linalg.norm(B.dot(u) - wrench) ** 2
        effort_cost = np.transpose(u).dot(thrust_cost).dot(u)
        return error_cost + effort_cost

    def obj_jacobian(u):
        error_jacobian = 2.0 * B.T.dot(B.dot(u) - wrench)
        effort_jacobian = np.transpose(u).dot(2 * thrust_cost)
        return error_jacobian + effort_jacobian

    minimization = minimize(method='slsqp',
                            fun=objective,
                            jac=obj_jacobian,
                            x0=(min_thrusts + max_thrusts) / 2,
                            bounds=zip(min_thrusts, max_thrusts),
                            tol=1e-5)
    return minimization

minimization = compute(wrench, 1e-4)
assert minimization.success


for i, label in enumerate(labels):
    print label + ': ', np.round(minimization.x[i], 3)

print "\nWrench: {}".format(np.round(wrench, 1))
print "Actual: {}\n".format(np.round(B.dot(minimization.x), 1))
