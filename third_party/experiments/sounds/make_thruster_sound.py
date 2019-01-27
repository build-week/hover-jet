'''This is an experiment inspired by Matt Vernacchia

Experiments with generating thruster-like sounds by generating largely
uniform noise over the frequency domain.

TODO: Empirically calibrate
'''
from scipy.io import wavfile
import os
import numpy as np


def mksin(tt, f, a, fs=44100.0):
    return a * np.sin(2.0 * np.pi * tt * fs / f).astype(np.float32)


def main():
    fs = 44100.0
    my_file = 'output.wav'
    tt = np.arange(0.0, 2.0, 1.0 / fs)
    signal = np.zeros(tt.shape, dtype=np.float32)

    for f in np.random.uniform(low=100.0, high=1500.0, size=(100)):
        samples = mksin(tt, f, 0.01)
        signal += samples

    wavfile.write(my_file, fs, signal)
    os.system('gnome-open {}'.format(my_file))

if __name__ == '__main__':
    main()
