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
            name, ln_data = ln.split(': ')
            ts, value = ln_data.split(',')
            data[name].append((float(ts), float(value)))

    print list(map(len, data.values()))

    return data


def main():
    path = '/home/jacob/repos/hover-jet/bin/poopy'
    data = parse(path)
    to_plot = [
        'mag_utesla_x',
        'mag_utesla_y',
        'mag_utesla_z',
        'turbine_rpm',
        'pump_voltage',
    ]

    for element in to_plot:
        assert element in data.keys()
        el_data = np.vstack(data[element])
        plt.plot(el_data[:, 0], el_data[:, 1], label=element)

    # for name, entries in data.items():
        # plt.plot(range(len(entries)), entries, label='Sensor: {}'.format(name))

    # plt.plot(range(len(data[7])), data[7], label='Sensor: {}'.format("Mag-x"))
    # plt.plot(range(len(data[8])), data[8], label='Sensor: {}'.format("Mag-y"))
    # plt.plot(range(len(data[9])), data[9], label='Sensor: {}'.format("Mag-z"))

    plt.title("Load Cell Output")
    plt.ylabel("Force (un-normalized)")
    plt.xlabel("Time (seconds)")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
