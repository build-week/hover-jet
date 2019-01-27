import numpy as np
from matplotlib import pyplot as plt


def circle():
    r = np.linspace(-np.pi, np.pi)
    d = np.vstack([np.sin(r), np.cos(r)])
    plt.plot(d[0, :], d[1, :])


def log_likelihood():
    r = np.linspace(-10.0, 10.0)

    C = np.array([
        1.0, 0.0,
        1.0, 1.0.
    ])

    l =

if __name__ == '__main__':



    plt.axis('equal')
    plt.show()
