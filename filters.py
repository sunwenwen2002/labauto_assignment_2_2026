import math

import numpy as np

'''
This file contains the implementation of the input shapers that can be used in robot_simulation.py.
The create_filters are implemented as single functions that return a function (the actual create_filter) that can be used as input_shaper.

The input shapers here implemented are:
- ZV: Zero Vibration shaper
- ZVD: Zero Vibration and Derivative shaper
- ZVDD: Zero Vibration and Double Derivative shaper
- EI: Extra Insensitive shaper (5% residual vibration)

The crane is considered as a pendulum and therefore as a second-order system.

The coefficients of each create_filter are calculated based on the following pendulum properties:
- Natural frequency of the pendulum (sqrt(g/L) based on the gravity and the pendulum length)
- Undamped system, decay ratio = 1  (damping="0.0" in the model.xml)
'''

# Data taken from the crane/model.xml model
L_PENDULUM = 0.736167857808297
GRAVITY = 9.81


def ZV(Tc, initial_reference=None):
    omega = math.sqrt(GRAVITY / L_PENDULUM)
    T = 2 * math.pi / omega

    # create_filter parameters
    amplitude = [0.5, 0.5]
    delay = [0.0, T / 2]

    return create_filter(amplitude, delay, Tc, initial_reference)


def ZVD(Tc, initial_reference=None):
    omega = math.sqrt(GRAVITY / L_PENDULUM)
    T = 2 * math.pi / omega
    
    # create_filter parameters
    amplitude = [0.25, 0.50, 0.25]
    delay = [0.0, T / 2, T]

    return create_filter(amplitude, delay, Tc, initial_reference)


def ZVDD(Tc, initial_reference=None):
    omega = math.sqrt(GRAVITY / L_PENDULUM)
    T = 2 * math.pi / omega

    # create_filter parameters
    amplitude = [0.125, 0.375, 0.375, 0.125]
    delay = [0.0, T / 2, T, 1.5 * T]

    return create_filter(amplitude, delay, Tc, initial_reference)


def EI(Tc, initial_reference=None, tollerance=0.05):
    omega = math.sqrt(GRAVITY / L_PENDULUM)
    T = 2 * math.pi / omega

    # EI coefficients with 5% residual vibration
    a1 = (1 + tollerance) / 4
    a2 = 1 - 2 * a1
    a3 = a1

    # create_filter parameters
    amplitude = [a1, a2, a3]
    delay = [0.0, T / 2, T]

    return create_filter(amplitude, delay, Tc, initial_reference)


def create_filter(amplitude, delay, Tc, initial_reference=None):
    # Convert delays from seconds to an integer number of samples.
    sample_delay = [int(round(delay / Tc)) for delay in delay]

    # The create_filter memory must be long enough to contain the oldest sample
    # needed to compute the create_filtered reference.
    memory_length = max(sample_delay) + 1

    if initial_reference is None:
        initial_reference = np.zeros(3)

    # Convert to numpy array and initialize the create_filter memory.
    # At the beginning, all memory samples are set equal to the initial reference.
    initial_reference = np.array(initial_reference, dtype=float)
    memory = [initial_reference.copy() for _ in range(memory_length)]

    # create_filter function that will then be used as input_shaper in robot_simulation.py.
    def filter(reference):
        reference = np.array(reference, dtype=float)

        # Add the new reference sample to memory.
        memory.append(reference.copy())

        filtred_reference = np.zeros_like(reference)

        # Compute the create_filtered reference.
        for A, delay in zip(amplitude, sample_delay):
            filtred_reference += A * memory[-1 - delay]

        # Remove the oldest sample to keep the memory at the desired length.
        memory.pop(0)

        return filtred_reference

    return filter
