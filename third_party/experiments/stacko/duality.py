from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def main():
	xls = np.arange(-4.0, 4.0, 0.25)
	lamls = np.arange(-4.0, 4.0, 0.25)
	xy = np.transpose([np.tile(xls, len(lamls)), np.repeat(lamls, len(xls))])


	x = xy[:, 0]
	lam = xy[:, 1]

	a = 0.0
	q = 5.0
	lagrangian = np.square(x) + (q * np.exp(-np.square(x)) * np.sin(1.0 + x)) - (lam * (x - a))


	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	ax.scatter(xy[:, 0], xy[:, 1], lagrangian)
	plt.show()


if __name__ == '__main__':
	main()
