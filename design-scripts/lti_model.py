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


    # Create state space model matrices
    A = np.zeros((4, 4))
    A[0, 2] = 1
    A[1, 3] = 1
    A[3, 0] = a_41
    print('System dynamics matrix A:')
    print(A)

    B = np.zeros(4)
    B[2] = b_3
    B[3] = b_4
    print('System input matrix B:')
    print(B)

    # Compute the eigenvalues of the system
    evals, evecs = np.linalg.eig(A)
    print('Dynamics matrix eigenvalues:')
    print(evals)

    # Create TF system model for vane aoa --> pitch-over
    sys_ang = signal.TransferFunction([b_3], [1, 0, 0])

    # Create TF system model for vane aoa --> translation
    sys_trans = signal.TransferFunction([b_4, 0, a_41 * b_3], [1, 0, 0, 0, 0])

    # Plot bode plot
    plot_bode(sys_trans)
    plt.suptitle('Bode plot for vane $\\alpha$ --> Lateral Translation $x$')
    # For comparison, plot the gain at which a 1 deg aoa sine
    # will drive a 0.05 meter position oscillation
    ref_x = 0.05
    ref_a = np.deg2rad(1)
    ref_gain = ref_x / ref_a
    plt.subplot(2, 1, 1)
    plt.axhline(y=20*np.log10(ref_gain),
        label='{:.2f} m / {:.1f} deg'.format(ref_x, np.rad2deg(ref_a)),
        color='red', linestyle='--')
    plt.legend()

    plot_bode(sys_ang)
    plt.suptitle('Bode plot for vane $\\alpha$ --> Pitch-over $\\theta$')

    # Plot "finesse" vs. frequency
    plt.figure()

    w, mag, phase = signal.bode(sys_trans, w=np.logspace(0, 2))
    gain = 10**(mag / 20)
    x_finess = gain * ref_a
    plt.loglog(w, x_finess, label='Position [m]')

    w, mag, phase = signal.bode(sys_ang, w=np.logspace(0, 2))
    gain = 10**(mag / 20)
    theta_finess = gain * ref_a
    plt.loglog(w, theta_finess, label='Pitch-over [rad]')
    plt.axhline(y=0.85, label='limit of linear model \n(10% error in $sin(x)=x$)', color='C1', linestyle=':')

    # Plot servo max speed for reference
    servo = 'MKS DS75K (4.8 V)'
    servo_speed = np.deg2rad(60) / 0.16    # Servo speed [units: radian second**-1].
    gear_ratio = 64 / 12.
    actuator_speed = servo_speed / gear_ratio
    w_max_act = actuator_speed / ref_a
    plt.axvline(x=w_max_act, label=('Actuator max $\\omega$\n'
        + '{:s} servo, {:.1f} gear ratio'.format(servo, gear_ratio)), color='black')
    
    plt.legend()
    plt.xlabel('$\\omega$ [rad/s]')
    plt.ylabel('Oscillation from {:.1f} deg $\\alpha$ sine'.format(np.rad2deg(ref_a)))
    plt.grid(True)

    plt.show()



if __name__ == '__main__':
    main()
