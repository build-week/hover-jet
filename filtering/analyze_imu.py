import numpy as np
from matplotlib import pyplot as plt


def main():
    path = '/home/jacob/repos/hover-jet/imu_data'
    data = np.genfromtxt(path, delimiter=',')

    accel = data[:, :3]
    avg_accel = np.mean(accel, axis=0)
    print "accel:", avg_accel
    print np.linalg.norm(avg_accel)

    print np.cov(accel.transpose())

    print accel[:, 0]
    # plt.scatter(accel[:, 0], accel[:, 1])
    plt.scatter(accel[:, 0], accel[:, 2])
    plt.show()

    gyro = data[:, 3:]
    print "gyro:", np.mean(gyro, axis=0)
    print np.cov(gyro.transpose())

if __name__ == '__main__':
    main()
