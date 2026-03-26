"""
Solve for gain given a desired force and a known voltage.

Chain:
  1. Force -> RPM  (invert the quadratic F(RPM))
  2. RPM, Voltage -> Gain  (invert the cubic polynomial RPM(gain, voltage))

Coefficients are loaded from motor_equations.yaml installed in the package
share directory (standard ROS 2 pattern).
"""

import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def _load_equations(package_name: str = "tauv_dronecan",
                    filename: str = "motor_equations.yaml") -> dict:
    """Load motor equation coefficients from the installed YAML file."""
    share_dir = get_package_share_directory(package_name)
    yaml_path = os.path.join(share_dir, "config", filename)
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


# ── Load coefficients once at import time ───────────────────────────────────
_EQ = _load_equations()



RPM_FROM_GAIN_VOLTAGE_POS = _EQ["rpm_from_gain_voltage"]["positive"]
RPM_FROM_GAIN_VOLTAGE_NEG = _EQ["rpm_from_gain_voltage"]["negative"]



def rpm_to_gain(
    target_rpm: float,
    voltage: float,
    gain_bounds: tuple = (0.0, 8191.0),
) -> float:
    """Invert RPM = f(gain, voltage) -> gain via np.roots.

    At fixed voltage the polynomial is quadratic in gain:
      (c_g2 + c_g2v*v)*g^2 + (c_g + c_gv*v + c_gv2*v^2)*g
        + (c0 + c_v*v + c_v2*v^2 + c_v3*v^3 - target_rpm) = 0
    """
    if abs(target_rpm) < 50:
        return 0.0

    c = RPM_FROM_GAIN_VOLTAGE_POS if target_rpm >= 0 else RPM_FROM_GAIN_VOLTAGE_NEG
    v = voltage

    A = c["c_g2"] + c["c_g2v"] * v
    B = c["c_g"] + c["c_gv"] * v + c["c_gv2"] * v**2
    C = c["c0"] + c["c_v"] * v + c["c_v2"] * v**2 + c["c_v3"] * v**3 - target_rpm

    roots = np.roots([A, B, C])
    real_roots = roots[np.isreal(roots)].real

    lo, hi = gain_bounds
    candidates = [float(r) for r in real_roots if lo <= r <= hi]

    if not candidates:
        return 0.0

    # If both are valid, pick the smaller (more reasonable) gain
    mult = 1 if target_rpm >= 0 else -1
    return min(candidates) * mult



# ── quick demo ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    test_cases = [
        (1000, 16.0, "positive rpm"),
        (-100, 16.0, "negative rpm"),
    ]

    for rpm, voltage, label in test_cases:
        print(f"\n--- {label}: RPM={rpm:.4f} N, V={voltage:.2f} V ---")
        gain = rpm_to_gain(rpm, voltage)
        print(f"  Solved gain: {gain:.2f}")
        print(f"  Simplified gain: {gain / 8191:.4f}")