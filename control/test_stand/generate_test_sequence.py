import numpy as np
from functools import partial
import os


MAX_SERVO_ANGLE_RAD = 1.1
MS_PER_TICK = 25
SEC_PER_MS = 1e-3
LENGTH_TICKS = 1000
ALL_INDICES = {0, 1, 2, 3}


def generate_yaml(command_list):
    assert command_list.ndim == 2
    assert command_list.shape[1] == 4
    text = "commands:\n"
    for cmd in command_list:
        text += "  - [{servo_0}, {servo_1}, {servo_2}, {servo_3}]\n".format(
            servo_0=cmd[0],
            servo_1=cmd[1],
            servo_2=cmd[2],
            servo_3=cmd[3]
        )

    return text


def write_file(text, path):
    with open(path, 'w') as f:
        f.write(text)


def set_zero():
    cmds = np.array([
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ])

    return cmds


def sine_wave_all(freq_hz, scaling_factor=np.array([1.0, 1.0, 1.0, 1.0])):
    ticks = np.arange(0.0, LENGTH_TICKS)

    millseconds = MS_PER_TICK * ticks
    seconds = millseconds * SEC_PER_MS

    angular_freq_radps = freq_hz * (2.0 * np.pi)

    result = MAX_SERVO_ANGLE_RAD * np.vstack([
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds)
    ]).transpose()

    return scaling_factor * result


def all_hold(target_angle):
    result = np.ones((LENGTH_TICKS, 4)) * target_angle
    return result


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--plot', action='store_true')
    args = parser.parse_args()

    tests = {
        "set_zero": set_zero(),
        "sine_wave_slow": sine_wave_all(freq_hz=0.1),
        "sine_wave_medium": sine_wave_all(freq_hz=0.5),
        "sine_wave_fast": sine_wave_all(freq_hz=1.0),
        "02_symmetric": sine_wave_all(freq_hz=0.1, scaling_factor=np.array([1.0, 0.0, 1.0, 0.0])),
        "13_symmetric": sine_wave_all(freq_hz=0.1, scaling_factor=np.array([0.0, 1.0, 0.0, 1.0])),
        "02_antisymmetric": sine_wave_all(freq_hz=0.1, scaling_factor=np.array([1.0, 0.0, -1.0, 0.0])),
        "13_antisymmetric": sine_wave_all(freq_hz=0.1, scaling_factor=np.array([0.0, 1.0, 0.0, -1.0])),
        "all_max": all_hold(target_angle=MAX_SERVO_ANGLE_RAD),
        "all_min": all_hold(target_angle=-MAX_SERVO_ANGLE_RAD),
    }

    for test_name, command_sequence in tests.items():
        if (args.plot):
            ticks = np.arange(0.0, command_sequence.shape[0])
            tt_milliseconds = ticks * MS_PER_TICK
            tt_seconds = tt_milliseconds * SEC_PER_MS
            for servo_index in range(4):
                plt.plot(tt_seconds, command_sequence[:, servo_index], label='Angle: {}'.format(servo_index))

            plt.ylim(np.array([-MAX_SERVO_ANGLE_RAD, MAX_SERVO_ANGLE_RAD]) * 1.1)
            plt.xlabel("Time From Start (Seconds)")
            plt.ylabel("Servo Angle (Radians)")
            plt.title("Command sequence for {}".format(test_name))
            plt.grid()
            plt.legend()
            plt.show()

        test_yaml_text = generate_yaml(command_sequence)
        test_file_name = "{}.yaml".format(test_name)
        output_path = os.path.join(os.getcwdu(), test_file_name)
        test_file_path = os.path.realpath(output_path)
        write_file(test_yaml_text, test_file_path)
        print("Writing to {} to {}".format(test_name, output_path))
