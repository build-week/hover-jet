"""Linearize model of the jet vehicle's lateral dynamics."""
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import pint
ureg = pint.UnitRegistry()


def plot_bode(sys):
    w, mag, phase = signal.bode(sys)
    plt.figure()
    ax1 = plt.subplot(2, 1, 1)
    plt.semilogx(w, mag)    # Bode magnitude plot
    plt.ylabel('Magnitude [dB]')
    plt.grid(True)
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)
    plt.semilogx(w, phase)  # Bode phase plot
    plt.xlabel('$\\omega$ [rad/s]')
    plt.ylabel('Phase [deg]')
    plt.grid(True)


def main():
    # Physical properties of vehicle.
    F_j = 160 * ureg.N   # Jet thrust.
    m = 10. * ureg.kg    # Vehicle mass.
    I = 2 / 5. * m * (0.2 * ureg.m)**2    # Vehicle moment of inertia about com in lateral plane.
    r_v = 0.2 * ureg.m    # Vane distance from vehicle center of mass.
    S_v = (20 * ureg.mm) * (15 * ureg.mm)    # Jet vane area.
    A_j = np.pi * (54 * ureg.mm / 2)**2    # Jet exit area.
    C_D0 = 0.05    # Jet vane drag coefficient minimum.

    # Coefficients of linear dynamic model.
    b_3_dimensioned = -np.pi * (S_v / A_j) * F_j * r_v / I
    b_3_dimensioned.ito(ureg.s**-2)
    b_3 = b_3_dimensioned.magnitude
    
    b_4_dimensioned = -np.pi * (S_v / A_j) * F_j / m
    b_4_dimensioned.ito(ureg.m * ureg.s**-2)
    b_4 = b_4_dimensioned.magnitude

    a_41_dimensioned = - (F_j / m) * (1 - 0.5 * (S_v / A_j) * C_D0)
    a_41_dimensioned.ito(ureg.m * ureg.s**-2)
    a_41 = a_41_dimensioned.magnitude

    # Create system model for vane aoa --> translation
    sys_trans = signal.TransferFunction([b_4, 0, a_41 * b_3], [1, 0, 0, 0, 0])

    # Plot bode plot
    plot_bode(sys_trans)
    plt.suptitle('Bode plot for vane $\\alpha$ --> Lateral Translation')
    plt.show()



if __name__ == '__main__':
    main()
