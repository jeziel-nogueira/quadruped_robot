# coding: utf-8
"""
utils.py

Math helpers and signal filters.
"""

import math

# ----------------------------
# MATH / UTILITIES
# ----------------------------
def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi
def clamp(x, a, b): return max(a, min(b, x))

# EMA smoother (per channel)
class EMAFilter:
    def __init__(self, alpha, init_val=0.0):
        self.alpha = alpha
        self.state = float(init_val)

    def update(self, value):
        self.state = self.alpha * value + (1.0 - self.alpha) * self.state
        return self.state

# Ramping helper: limits angular velocity
class SlewLimiter:
    def __init__(self, max_deg_per_sec, init_deg=0.0):
        self.max_dps = max_deg_per_sec
        self.state = float(init_deg)

    def update(self, target_deg, dt):
        max_step = self.max_dps * dt
        diff = target_deg - self.state
        if abs(diff) <= max_step:
            self.state = target_deg
        else:
            self.state += math.copysign(max_step, diff)
        return self.state
