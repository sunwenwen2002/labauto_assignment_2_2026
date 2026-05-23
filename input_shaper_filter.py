import math
from enum import Enum

import numpy as np

'''
This file contains the implementation of the input shaper class that can be used in robot_simulation.py.

The input shapers here implemented are:
- ZV: Zero Vibration shaper
- ZVD: Zero Vibration and Derivative shaper
- ZVDD: Zero Vibration and Double Derivative shaper
- EI: Extra Insensitive shaper (5% residual vibration)

The crane is considered as a pendulum and therefore as a second-order system.

The coefficients of each create_filter are calculated based on the following pendulum properties:
- Natural frequency of the pendulum (sqrt(g/L) based on the gravity and the pendulum length)
- Damped system, damping ratio = XI
'''


class ShaperType(Enum):
    ZV = "ZV"
    ZVD = "ZVD"
    ZVDD = "ZVDD"
    EI = "EI"


class InputShaperFilter:

    L_PENDULUM = 0.736167857808297
    GRAVITY = 9.81
    XI = 3.2090757462851513e-06 # Damping ratio estimated from the impulse response of the system.

    omega_n = math.sqrt(GRAVITY / L_PENDULUM)
    omega_d = omega_n * math.sqrt(1 - XI**2)
    T = 2 * math.pi / omega_d
    K = math.exp((-XI * math.pi) / math.sqrt(1 - XI**2))

    def __init__(self, Tc, filter_type, initial_reference=None, tolerance=0.05):
        self.Tc = Tc
        self.filter_type = filter_type

        if initial_reference is None:
            initial_reference = np.zeros(3)

        self.initial_reference = np.array(initial_reference, dtype=float)

        # Parameters to be set later based on the chosen filter.
        self.amplitude = []
        self.delay = []
        self.sample_delay = []
        self.memory = []

        # Set the filter based on the chosen type.
        self.set_filter(filter_type, tolerance)

    def set_filter(self, filter_type, tolerance=0.05):
        # Create the filter based on the chosen type.
        if filter_type == ShaperType.ZV:
            self.ZV()
        elif filter_type == ShaperType.ZVD:
            self.ZVD()
        elif filter_type == ShaperType.ZVDD:
            self.ZVDD()
        elif filter_type == ShaperType.EI:
            self.EI(tolerance)
        else:
            raise ValueError("Unknown input shaper filter type.")

    def ZV(self):
        amplitude = [
            1 / (1 + self.K),
            self.K / (1 + self.K),
        ]
        delay = [0.0, self.T / 2]

        self.create_filter(amplitude, delay)

    def ZVD(self):
        amplitude = [
            1 / (1 + self.K)**2,
            2 * self.K / (1 + self.K)**2,
            self.K**2 / (1 + self.K)**2,
        ]
        delay = [0.0, self.T / 2, self.T]

        self.create_filter(amplitude, delay)

    def ZVDD(self):
        amplitude = [
            1 / (1 + self.K)**3,
            3 * self.K / (1 + self.K)**3,
            3 * self.K**2 / (1 + self.K)**3,
            self.K**3 / (1 + self.K)**3,
        ]
        delay = [0.0, self.T / 2, self.T, 1.5 * self.T]

        self.create_filter(amplitude, delay)

    def EI(self, tolerance=0.05):
        # EI coefficients with 5% residual vibration
        a1 = (1 + tolerance) / (1 + self.K)**2
        a2 = 2 * self.K * (1 - tolerance) / (1 + self.K)**2
        a3 = self.K**2 * (1 + tolerance) / (1 + self.K)**2

        amplitude = [a1, a2, a3]
        delay = [0.0, self.T / 2, self.T]

        self.create_filter(amplitude, delay)
    
    def create_filter(self, amplitude, delay):
        self.amplitude = amplitude
        self.delay = delay

        # Convert delays from seconds to an integer number of samples.
        self.sample_delay = [int(round(delay / self.Tc)) for delay in self.delay]

        # The create_filter memory must be long enough to contain the oldest sample
        # needed to compute the create_filtered reference.
        memory_length = max(self.sample_delay) + 1

        # Convert to numpy array and initialize the create_filter memory.
        # At the beginning, all memory samples are set equal to the initial reference.
        self.memory = [self.initial_reference.copy() for _ in range(memory_length)]

    def filter(self, reference):
        reference = np.array(reference, dtype=float)

        # Add the new reference sample to memory.
        self.memory.append(reference.copy())

        filtered_reference = np.zeros_like(reference)

        # Compute the create_filtered reference.
        for A, delay in zip(self.amplitude, self.sample_delay):
            filtered_reference += A * self.memory[-1 - delay]

        # Remove the oldest sample to keep the memory at the desired length.
        self.memory.pop(0)

        return filtered_reference
