"""Landing Gear design script for Hover Jet"""
import numpy as np
import pint

ureg = pint.UnitRegistry()

material_data = {
    'polycarbonate': {
        'Young Modulus' : 2.38 * ureg.GPa,
        'Density': 1200 * ureg.kg * ureg.m**-3,
    },
    'Ti grade 2': {
        # Source: MatWeb, Titanium Grade 2, Annealed 
        # http://www.matweb.com/search/datasheet.aspx?matguid=49a4b764217b44ee953205822af5fbc9
        'Young Modulus' : 102 * ureg.GPa,
        'Density': 4510 * ureg.kg * ureg.m**-3,
    },
}


def main():
    print('*** Landing Leg Analysis ***')
    #### Model Parameters ####
    # Leg properties
    leg_angle = np.deg2rad(45)   # Leg angle from horizontal
    leg_len = 0.3 * ureg.meter    # Leg length
    # Leg material Young's Modulus
    material = 'Ti grade 2'
    E = material_data[material]['Young Modulus']
    # Tube dimensions
    d_o = 0.50 * ureg.inch    # tube outer diameter
    d_i = 0.43 * ureg.inch    # tube inner diameter
    # Tube's second moment of area in bending
    I = np.pi  / 64 * (d_o**4 - d_i**4)
    # Single leg beam bending stiffness
    k_bend = 3 * E * I / leg_len**3

    # Vehicle properties
    m_v = 10 * ureg.kg    # Vehicle mass

    # Crash properties
    n_legs = 2    # Number of legs in contact
    v_crash = 5 * ureg.m / ureg.s    # crash initial velocity

    # Print inputs:
    print('Inputs:')
    print('\tLeg material = {:s}'.format(material))
    print('\tLeg length = {:~P}'.format(leg_len))
    print('\tLeg diameters = {:~.3P} OD, {:~.3P} ID'.format(d_o.to(ureg.mm), d_i.to(ureg.mm)))
    print('\tCrash velocity = {:~P}'.format(v_crash))
    print('\n')

    # Leg mass
    m_leg = (leg_len * np.pi / 4 * (d_o**2 - d_i**2)
        * material_data[material]['Density'])
    print('\tLeg mass, each = {:~.4P}'.format(m_leg.to(ureg.gram)))
    print('\n')

    #### Analytic solution #####
    # Simple dynamics model
    # Assumptions:
    #  1) Ignore gravity
    #  2) Leg contact w/ ground is frictionless, so that the contact
    #     force is normal to the ground
    #  3) The landing legs are massless
    #  4) The legs stay in the linear (constant effective stiffness) regime
    #     even as they deflect. (This becomes invalid at large deflections,
    #     because the landing leg angle changes as the legs deflect,
    #     and the effective stiffness of the legs to vertical displacement
    #     depends on the leg angle.)
    #  5) n_legs contact the ground at the same instant, the vehicle has vertical
    #     velocity, and only moves in the vertical direction.
    #
    # Let z be the vertical displacement of the vehicle (zero legs undeformed, positive down).
    # Let F_v be the force on the vehicle due to the legs, and let
    # k_legs = F_v / z be the effective stiffness of the landing legs.
    #
    # Newton's Law gives:
    #     F_v = - k_legs z = m_v (d^2 z)/(dt^2)
    #
    # This is a simple harmonic oscillator.
    # We also have the initial conditions:
    #    Let t = 0 be the moment the legs first contact the ground.
    #    z = 0 at t = 0
    #    dz/dt = v_crash at t = 0
    # We can compute the maximum vertical displacement and vertical
    # acceleration from the equations of motion of the oscillator.

    # Vertical displacement to bottom out the landing legs
    z_bottom = leg_len * np.sin(leg_angle)

    # Effective landing legs stiffness
    # This is the ratio of (vertical force on the vehicle
    # from the land legs) / (vertical displacement of the vehicle, z)
    k_legs = n_legs * k_bend / (np.cos(leg_angle))**2


    # Landing leg fundamental frequency [units: radian second**-1]
    omega_legs = (k_legs / m_v)**0.5
    omega_legs.ito(ureg.s**-1)

    # Maximum displacement
    z_max = v_crash / omega_legs
    z_max.ito(ureg.m)

    # maximum acceleration
    accel_max = v_crash * omega_legs

    # Print results:
    print('Simple Harmonic Oscillator Results:')
    print('\tFundamental frequency = {:~.3P}'.format(
        (omega_legs * ureg.rad).to(ureg.hertz)))
    print('\tMax. vertical displacement = {:~.2P} (/ {:~.2P} to bottom out)'.format(
        z_max, z_bottom))
    print('\tMax. acceleration = {:.4~P} ({:.2f} g_0)'.format(
        accel_max, (accel_max / ureg.g_0).to('dimensionless').magnitude))
    print('\n')

if __name__ == '__main__':
    main()
