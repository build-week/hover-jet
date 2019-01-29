import numpy as np


def main():
    np.set_printoptions(linewidth=120)

    J = np.array([
        [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    ])

    JtJ = J.dot(J.transpose())

    print JtJ
    print np.linalg.cholesky(JtJ)
    # print np.linalg.inv(JtJ)


if __name__ == '__main__':
    main()
