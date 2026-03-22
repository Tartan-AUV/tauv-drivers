"""
Motor equations relating Force, RPM, Gain, and Voltage.

Positive RPM:
  F = 4.557e-07*RPM^2 + -1.402e-04*RPM + 6.364e-02

Negative RPM:
  F = -3.849e-07*RPM^2 + -1.938e-04*RPM + -1.015e-01

Positive side RPM(gain, voltage) (R²=0.9969):
  RPM = -1415.3219 + 0.243691*g + 0.780504*v + 0.000030*g^2
        + 0.023399*g*v + 12.906228*v^2 + -0.000004*g^2*v
        + 0.000359*g*v^2 + -0.468978*v^3

Negative side RPM(gain, voltage) (R²=0.9973):
  RPM = -523.9110 + 0.384959*g + 0.297113*v + 0.000009*g^2
        + 0.007116*g*v + 4.904108*v^2 + -0.000003*g^2*v
        + 0.000783*g*v^2 + -0.179616*v^3
"""

# --- Force from RPM (quadratic) ---
# F = a*RPM^2 + b*RPM + c

FORCE_FROM_RPM_POS = {"a": 4.557e-07, "b": -1.402e-04, "c": 6.364e-02}
FORCE_FROM_RPM_NEG = {"a": -3.849e-07, "b": -1.938e-04, "c": -1.015e-01}

# --- RPM from Gain and Voltage (cubic polynomial) ---
# RPM = c0 + c_g*g + c_v*v + c_g2*g^2 + c_gv*g*v + c_v2*v^2
#        + c_g2v*g^2*v + c_gv2*g*v^2 + c_v3*v^3

RPM_FROM_GAIN_VOLTAGE_POS = {
    "c0":    -1415.3219,
    "c_g":       0.243691,
    "c_v":       0.780504,
    "c_g2":      0.000030,
    "c_gv":      0.023399,
    "c_v2":     12.906228,
    "c_g2v":    -0.000004,
    "c_gv2":     0.000359,
    "c_v3":     -0.468978,
}

RPM_FROM_GAIN_VOLTAGE_NEG = {
    "c0":     +523.9110,
    "c_g":       -0.384959,
    "c_v":       -0.297113,
    "c_g2":      -0.000009,
    "c_gv":      -0.007116,
    "c_v2":      -4.904108,
    "c_g2v":    0.000003,
    "c_gv2":     -0.000783,
    "c_v3":     0.179616,
}