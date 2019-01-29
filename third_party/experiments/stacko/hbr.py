import numpy as np
from scipy import stats
from matplotlib import pyplot as plt


if __name__ == '__main__':
    a = stats.norm(loc=17.0, scale=0.1)

    x = np.linspace(15.0, 21.0, 1000)
    p = a.pdf(x)

    print a.pdf(13.0)

    plt.plot(x, p)
    plt.show()
