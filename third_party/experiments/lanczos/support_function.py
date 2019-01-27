import numpy as np
from matplotlib import pyplot as plt


def ax3d(title='3D'):
    from mpl_toolkits.mplot3d import Axes3D  # noqa
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax


def make_grid(x_range, y_range):
    grid = np.meshgrid(x_range, y_range)
    return np.hstack(grid).transpose().reshape((2, -1))


def support_function_line_segment(x):
    a = np.array([1.0, 0.0])

    h = a.dot(x)

    ax = ax3d('A-A')
    ax.scatter(x[0, :], x[1, :], h)

    ax.set_xlim([-4.0, 4.0])
    ax.set_ylim([-4.0, 4.0])
    ax.set_zlim([-4.0, 4.0])

    plt.show()


def main():
    x_range = np.arange(-3.0, 3.0, 0.9)
    y_range = np.arange(-3.0, 3.0, 0.9)
    x = make_grid(x_range, y_range)

    support_function_line_segment(x)


if __name__ == '__main__':
    main()
