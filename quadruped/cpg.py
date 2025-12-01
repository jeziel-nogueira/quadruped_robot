# coding: utf-8
"""
cpg.py

Hopf Oscillator and CPG Network.
"""

import math
import config

# ----------------------------
# HOPF OSCILLATOR (simple)
# ----------------------------
class HopfOscillator:
    def __init__(self, mu=1.0, omega=2*math.pi*config.CPG_FREQ, alpha=10.0, dt=config.CPG_DT, init=None):
        self.mu = mu
        self.omega = omega
        self.alpha = alpha
        self.dt = dt
        if init is None:
            self.x = 0.01
            self.y = 0.0
        else:
            self.x, self.y = init

    def step(self, coupling=(0.0, 0.0)):
        # Hopf oscillator (cartesian form) with simple coupling
        r2 = self.x*self.x + self.y*self.y
        dx = self.alpha*(self.mu - r2)*self.x - self.omega*self.y + coupling[0]
        dy = self.alpha*(self.mu - r2)*self.y + self.omega*self.x + coupling[1]
        self.x += dx * self.dt
        self.y += dy * self.dt
        phase = math.atan2(self.y, self.x)
        amp = math.sqrt(max(0.0, self.x*self.x + self.y*self.y))
        return phase, amp

# ----------------------------
# CPG NETWORK (simplified for bench tests)
# ----------------------------
class SimpleCPGNetwork:
    def __init__(self, legs=4, joints_per_leg=3, freq=config.CPG_FREQ):
        self.legs = legs
        self.joints = joints_per_leg
        self.osc = [[HopfOscillator(mu=1.0, omega=2*math.pi*freq, dt=config.CPG_DT) for _ in range(joints_per_leg)] for _ in range(legs)]
        # phase offsets per leg for hip oscillator (simple gait presets)
        self.phase_offsets = {
            'stand': [0.0, 0.0, 0.0, 0.0],
            'walk':  [0.0, 0.5, 0.25, 0.75],  # as you had before (fractions of cycle)
            'trot':  [0.0, 0.5, 0.5, 0.0],
        }
        self.set_gait('walk')

    def set_gait(self, gait):
        frac = self.phase_offsets.get(gait, self.phase_offsets['walk'])
        # convert to radians phase offsets and store in a matrix for hips; simple approach
        self.hip_phase_offsets = [f * 2*math.pi for f in frac]

    def step(self):
        """Advance all oscillators and return per-oscillator phase and amplitude."""
        phases = [[0.0]*self.joints for _ in range(self.legs)]
        amps = [[0.0]*self.joints for _ in range(self.legs)]
        # For simplicity we don't implement complex inter-oscillator coupling in this template.
        for i in range(self.legs):
            for j in range(self.joints):
                p, a = self.osc[i][j].step()
                # apply leg/hip offset only to hip oscillator j==0
                if j == 0:
                    p += self.hip_phase_offsets[i]
                phases[i][j] = p
                amps[i][j] = a
        return phases, amps
