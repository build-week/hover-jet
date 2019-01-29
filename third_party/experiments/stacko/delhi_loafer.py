import numpy as np
from matplotlib import pyplot as plt
import sympy
import scipy.linalg as scla


def ax3d():
    from mpl_toolkits.mplot3d import Axes3D  # noqa
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax


def skew3(w):
    return np.cross(np.identity(3), w)


if __name__ == '__main__':
    f = "/home/jacob/repos/experiments/bin/bloop"
    data = np.loadtxt(f)
    norms = np.linalg.norm(data, axis=1)
    a = data[(norms != 0) & (norms < 1e2), :]

    ray = np.array([1.0, 0.0, 0.0])

    rays = []
    for row in a:
        R = scla.expm3(skew3(row[3:]))
        direction = R.dot(ray)
        rays.append(direction)

    rays = np.vstack(rays) * -200.0

    ax = ax3d()
    ax.scatter(a[:, 0], a[:, 1], a[:, 2])

    print rays.shape
    print a.shape

    ax.quiver(a[:, 0], a[:, 1], a[:, 2], rays[:, 0], rays[:, 1], rays[:, 2])
    plt.show()
