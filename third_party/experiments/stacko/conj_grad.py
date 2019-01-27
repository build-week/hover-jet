import numpy as np
from matplotlib import pyplot as plt


def ax3d():
    from mpl_toolkits.mplot3d import Axes3D  # noqa
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax


def draw_q(Q, b):
    xx = np.linspace(-2.0, 2.0, 100)
    yy = np.linspace(-2.0, 2.0, 100)

    XX, YY = np.meshgrid(xx, yy)

    arr = np.vstack((XX, YY)).reshape((2, -1))
    v = []
    for n in range(len(arr.transpose())):
        u = arr[:, n]
        v.append(0.5 * (u.dot(Q).dot(u)) - b.dot(u))

    vals = np.vstack(v)
    plt.contour(XX, YY, vals.reshape(100, 100), 50)


def visualize_cgrad():
    x0 = np.array([
        [-0.3295, 0.536459],
        [0.82965, -0.471468],
        [1.05311, -0.308429],
    ])

    Q = np.array([
        [0.676846, 0.105097],
        [0.105097, 1.043710],
    ])

    b = np.array([0.680375, -0.211234])
    # draw_q(Q, b)
    plt.scatter(x0[:, 0], x0[:, 1])
    # plt.show()


def visualize_gd():
    x0 = np.array([-0.3295, 0.536459])

    Q = np.array([
        [0.676846, 0.105097],
        [0.105097, 1.043710],
    ])

    # Q = np.eye(2) * 0.1

    b = np.array([0.680375, -0.211234])

    x_hist = [np.copy(x0)]
    for k in range(61):
        x0 -= 0.7 * (Q.dot(x0) - b)
        x_hist.append(np.copy(x0))
        print 'x', x0, 'r', Q.dot(x0) - b

    print '--'
    print np.linalg.solve(Q, x0)

    npx = np.vstack(x_hist)
    plt.scatter(npx[:, 0], npx[:, 1])
    draw_q(Q, b)
    print npx[:, 0].shape
    plt.show()


if __name__ == '__main__':
    visualize_cgrad()
    visualize_gd()
