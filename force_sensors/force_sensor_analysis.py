import numpy as np
from matplotlib import pyplot as plt
from collections import defaultdict


def main2():
    data = np.genfromtxt('/home/jacob/repos/hover-jet/bin/poopy2')
    plt.plot(range(len(data)), data)
    plt.show()


def parse(file):
    data = defaultdict(list)
    with open(file) as f:
        for ln in f:
            data[int(ln[0])].append(float(ln[3:]))

    print list(map(len, data.values()))

    return data


def main():
    path = '/home/jacob/repos/hover-jet/bin/poopy'
    data = parse(path)
    for name, entries in data.items():
        plt.plot(range(len(entries)), entries, label='Sensor: {}'.format(name))
    plt.show()


if __name__ == '__main__':
    main()
