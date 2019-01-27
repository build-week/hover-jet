import numpy as np
from matplotlib import pyplot as plt


def huber(x, k):
    abs_x = np.abs(x)

    res = np.zeros_like(x)

    x_gt_k = abs_x > k
    x_lte_k = ~x_gt_k

    res[x_gt_k] = k * (abs_x[x_gt_k] - (k * 0.5))
    res[x_lte_k] = (x[x_lte_k] * x[x_lte_k]) * 0.5
    return res


def main():
    t = np.linspace(-5.0, 5.0, 1000)

    k = 1.0
    ht = huber(t, k)
    # plt.plot(t, ht)

    plt.plot(t, np.exp(-ht))
    plt.plot(t, np.exp(-(t * t)))

    plt.show()


if __name__ == '__main__':
    main()
