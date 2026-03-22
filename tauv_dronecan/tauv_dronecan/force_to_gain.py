"""
Solve for gain given a desired force and a known voltage.

Chain:
  1. Force -> RPM  (invert the quadratic F(RPM))
  2. RPM, Voltage -> Gain  (invert the cubic polynomial RPM(gain, voltage))
"""

import numpy as np
from tauv_dronecan.equations import (
    FORCE_FROM_RPM_POS,
    FORCE_FROM_RPM_NEG,
    RPM_FROM_GAIN_VOLTAGE_POS,
    RPM_FROM_GAIN_VOLTAGE_NEG,
)
FORCE_DEADBAND = 0.03  # Forces with abs value below this clip to gain=0


def _force_to_rpm(force: float) -> float:
    """Invert F = a*RPM^2 + b*RPM + c  ->  RPM via np.roots."""
    coefs = FORCE_FROM_RPM_POS if force >= 0 else FORCE_FROM_RPM_NEG
    a, b, c = coefs["a"], coefs["b"], coefs["c"]
 
    # a*RPM^2 + b*RPM + (c - F) = 0
    roots = np.roots([a, b, c - force])
    real_roots = roots[np.isreal(roots)].real
 
    if len(real_roots) == 0:
        return 0;
        raise ValueError(
            f"No real RPM solution for F={force:.4f} "
        )
 
    # Pick the physically meaningful root
    if force >= 0:
        return float(np.max(real_roots))
    else:
        return float(np.min(real_roots))
 

def _rpm_to_gain(
    target_rpm: float,
    voltage: float,
    
    gain_bounds: tuple = (0.0, 65535.0),
) -> float:
    """Invert RPM = f(gain, voltage) -> gain via np.roots.
 
    At fixed voltage the polynomial is quadratic in gain:
      (c_g2 + c_g2v*v)*g^2 + (c_g + c_gv*v + c_gv2*v^2)*g
        + (c0 + c_v*v + c_v2*v^2 + c_v3*v^3 - target_rpm) = 0
    """
    if(abs(target_rpm)<50):
        return 0.0
    
    c = RPM_FROM_GAIN_VOLTAGE_POS if target_rpm>=0 else RPM_FROM_GAIN_VOLTAGE_NEG
    v = voltage
 
    A = c["c_g2"] + c["c_g2v"] * v
    B = c["c_g"] + c["c_gv"] * v + c["c_gv2"] * v**2
    C = c["c0"] + c["c_v"] * v + c["c_v2"] * v**2 + c["c_v3"] * v**3 - target_rpm
 
    roots = np.roots([A, B, C])
    real_roots = roots[np.isreal(roots)].real
 
    lo, hi = gain_bounds
    candidates = [float(r) for r in real_roots if lo <= r <= hi]
 
    if not candidates:
        raise ValueError(
            f"No gain solution in [{lo}, {hi}] for RPM={target_rpm:.1f}, "
            f"V={voltage:.2f}. Roots: {real_roots}"
        )
 
    # If both are valid, pick the smaller (more reasonable) gain
    mult = 1 if target_rpm >= 0 else -1
    return min(candidates)*mult
 
 
 
 
def force_voltage_to_gain(
    force: float,
    voltage: float,
    gain_bounds: tuple = (0.0, 8191.0),
) -> float:
   
    force =force/9.81 # Convert from N to kgf, since the equations are based on kgf
    if abs(force) < FORCE_DEADBAND:
        return 0.0
    if force>40:
        return 0.0
 
    target_rpm = _force_to_rpm(force)
    print(f"  -> Target RPM: {target_rpm:.1f}")
    return _rpm_to_gain(target_rpm, voltage, gain_bounds)
 
# ── quick demo ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    test_cases = [
        ( 1, 16.0, "positive force"),
        (-10, 16.0, "negative force"),

    ]

    for force, voltage, label in test_cases:
        print(f"\n--- {label}: F={force:.4f} N, V={voltage:.2f} V ---")
        gain = force_voltage_to_gain(force, voltage)
        print(f"  Solved gain: {gain:.2f}")
        print(f" Simplifed gain: {gain/8191:.4f} ")

        # if gain > 0:
        #     side = "pos" if force >= 0 else "neg"
        #     rpm = _rpm_from_gain_voltage(gain, voltage, side)
        #     coefs = FORCE_FROM_RPM_POS if side == "pos" else FORCE_FROM_RPM_NEG
        #     print(f"  -> RPM:         {rpm:.1f}")
