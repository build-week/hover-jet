import numpy as np


def main():

    nx = 2
    ny = 3
    nz = 4

    def do_it(i):
        return [i % nx, (i / nx) % ny, i / (nx * ny)]

    def undo_it(x, y, z):
        # return z + ny * (y + nx * x)
        # return x + nx * (x + ny * y)
        # return (x * nx * ny) + (y * ny) + z
        # return (z * nx * ny) + (nx * y) + x
        return nx * ((nx * z) + y) + x

    for i in range(nx * ny * nz):
        xyz = do_it(i)
        print '{}: {}'.format(i, xyz)
        uuzz = np.array(xyz)
        print '{}: {}'.format(i, undo_it(*uuzz))
        # print undo_it(*xyz)


main()
