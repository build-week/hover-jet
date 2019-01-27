from matplotlib import pyplot as plt
import numpy as np


def ax3d():
    from mpl_toolkits.mplot3d import Axes3D  # noqa
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax


def vline(ax, pt, h):
    plt.plot([pt[0], pt[0]], [pt[1], pt[1]], [-h, h])


def nullspace(A):
    atol = 1e-13
    rtol = 0.0
    A = np.atleast_2d(A)
    u, s, vh = np.linalg.svd(A)
    tol = max(atol, rtol * s[0])
    nnz = (s >= tol).sum()
    ns = vh[nnz:].conj().T
    return ns


if __name__ == '__main__':
    r = np.linspace(-10.0, 10.0)
    X, Y = np.meshgrid(r, r)
    xy = np.vstack(map(np.ravel, np.meshgrid(r, r)))

    ax = ax3d()
    ax.plot_surface(X, Y, 36 * X * Y)

    # for m in np.arange(-10.0, 10.0, 1.0):
    m = 1.0
    y_proj = (m - 3.0 * r) / 6.0
    f_ry = 36 * r * y_proj

    ax.plot(r, y_proj, f_ry)
    exex = (1 / 6.0) * m
    vline(ax, np.array([exex, (m - 3.0 * exex) / 6.0]), 1000.0)

    # plt.show()

    H = np.array([
        [0.0, 36.0],
        [36.0, 0.0]
    ])

    # A = np.array([
    #     [3.0],
    #     [6.0],
    # ])

    # Anull_a = np.array([
    #     [-1.0],
    #     [1.0 / 2.0]
    # ])

    A = np.array([
        [1.0],
        [9.0],
    ])

    Anull_a = np.array([
        [-1.0],
        [1.0 / 9.0],
    ])

    # Anull_a = np.array([
    #     [-1.0],
    #     [1.0 / 2.0]
    # ])

    print np.linalg.norm(Anull_a)
    print np.linalg.norm(Anull_a, 1)

    # Anull_a = np.array([
    #     [-2.0 / 3.0],
    #     [-2.0 / 3.0],
    # ])

    print '---'

    Anull = Anull_a / np.linalg.norm(Anull_a)
    print Anull.transpose().dot(A)
    print Anull

    # print nullspace(A)

    # He = np.zeros((3, 3))
    # He[:2, :2] = H

    print Anull_a.transpose().dot(H).dot(Anull_a)
    # print A.transpose().dot(He).dot(A)
