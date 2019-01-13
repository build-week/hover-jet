import numpy as np
from functools import partial
import os

MAX_ANGLE_RAD = 0.2268
COMMAND_PERIOD_MS = 25
LENGTH_TICKS = 10000
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


def sine_wave_all(freq_hz):
    milliseconds = np.arange(0.0, LENGTH_TICKS, COMMAND_PERIOD_MS)

    seconds = milliseconds * 1e-3

    angular_freq_radps = freq_hz / (2.0 * np.pi)

    return np.vstack([
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds)
    ]).transpose()


def same(included, freq_hz=0.1):
    milliseconds = np.arange(0.0, LENGTH_TICKS, COMMAND_PERIOD_MS)

    seconds = milliseconds * 1e-3

    angular_freq_radps = freq_hz / (2.0 * np.pi)

    result = np.vstack([
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        -np.sin(angular_freq_radps * seconds),
        -np.sin(angular_freq_radps * seconds)
    ]).transpose()

    not_incls = ALL_INDICES - set(included)
    for not_incl in not_incls:
        result[:, not_incl] == 0.0

    return result


def opposite(included, freq_hz=0.1):
    milliseconds = np.arange(0.0, LENGTH_TICKS, COMMAND_PERIOD_MS)

    seconds = milliseconds * 1e-3
    angular_freq_radps = freq_hz / (2.0 * np.pi)

    result = np.vstack([
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds),
        np.sin(angular_freq_radps * seconds)
    ]).transpose()

    not_incls = ALL_INDICES - set(included)

    for not_incl in not_incls:
        result[:, not_incl] == 0.0

    return result


def all_hold(target_angle):
    result = np.ones((LENGTH_TICKS, 4))
    result *= target_angle
    return result


if __name__ == '__main__':
    tests = {
        "set_zero": set_zero,
        "sine_wave_slow": partial(sine_wave_all, freq_hz=0.1),
        "sine_wave_medium": partial(sine_wave_all, freq_hz=1.0),
        "sine_wave_fast": partial(sine_wave_all, freq_hz=3.0),

        "02_same": partial(same, included=(0, 2)),
        "13_same": partial(same, included=(1, 3)),

        "02_opposite": partial(opposite, included=(0, 2)),
        "13_opposite": partial(opposite, included=(1, 3)),

        "all_max": partial(all_hold, target_angle=MAX_ANGLE_RAD),
        "all_min": partial(all_hold, target_angle=-MAX_ANGLE_RAD),
    }

    for test_name, generate_cmds in tests.items():
        test_cmds = generate_cmds()
        test_yaml_text = generate_yaml(test_cmds)

        test_file_name = "{}.yaml".format(test_name)
        write_file(test_yaml_text, test_file_name)

        output_path = os.path.join(os.getcwdu(), test_file_name)
        real_path = os.path.realpath(output_path)
        print("Writing to {} to {}".format(test_name, output_path))
