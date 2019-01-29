import numpy as np
from matplotlib import pyplot as plt


def main():
    Q = np.identity(2)

    level_set = 2.0

    x = np.array([1.0, 1.0])
    gradient = 2.0 * Q.dot(x)
    print (x.dot(Q.dot(x)))

    print np.linalg.eig(np.vstack([gradient, np.zeros(2)]))

    eigen_vals, eigen_vecs = np.linalg.eig(Q)
    print(eigen_vals)
    print(eigen_vecs)


if __name__ == '__main__':
    main()
