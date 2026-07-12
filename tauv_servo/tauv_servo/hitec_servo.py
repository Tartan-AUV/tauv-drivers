"""
hitec_servo.py — Hitec CAN Servo Control Library
==================================================
Controls Hitec MDB961WP-CAN servos via proprietary CAN 2.0A register protocol.

Verified against: CAN 2.0A/B / DroneCAN Servo Control Protocol Manual Rev 2.03_EN
Servo spec:        MDB961WP-CAN — 8-32V, ±150° programmable, 4096 steps = 90°

IMPORTANT — Position limits:
    The servo has configurable position limits. The MDB961WP datasheet says the
    default is ±60° from center (120°-240°). Max programmable is ±150° (30°-330°).
    If you command a position OUTSIDE the limits, the servo SILENTLY IGNORES the
    command — it does NOT clamp, it just doesn't move.

    Call set_limits(30, 330) and save() to unlock the full ±150° range, or use
    setup_servo() which does this automatically.

Requirements:
    pip install python-can

CAN interface setup (run once after boot):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000 sample-point 0.500
    sudo ip link set can0 up

Usage:
    from hitec_servo import HitecBus

    bus = HitecBus("can0")
    s = bus.servo(1)

    # Absolute (0-360°, 180° is center)
    s.move(180.0)

    # Relative to center (-150 to +150°)
    s.move_relative(-90.0)    # 90° left of center = 90° absolute

    print(s.position())           # absolute degrees
    print(s.position_relative())  # offset from center
    print(s.telemetry())
    bus.close()
"""

import can
import time
import threading
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

# =============================================================================
# Constants — verified against protocol manual Rev 2.03 pages 12-15
# =============================================================================

# Position conversion: manual section 2-3.2, 2-5.1
# "resolution is 4096 = 90°", range 0-16383
_STEPS_PER_DEG = 4096.0 / 90.0   # ~45.511 steps per degree
_DEG_PER_STEP = 90.0 / 4096.0    # ~0.02197 degrees per step
_MAX_RAW = 16383                  # max position value (manual p.12)
_CENTER_DEG = 180.0               # center in degrees

def _deg_to_raw(degrees: float) -> int:
    """Convert absolute degrees (0-360) to raw steps (0-16383). Manual: 4096 = 90°."""
    raw = round(degrees * _STEPS_PER_DEG)
    return max(0, min(_MAX_RAW, raw))

def _s16(raw: int) -> int:
    """Reinterpret a 16-bit register as signed. Velocity, temperature and
    current are all two's-complement (velocity goes negative when the servo
    runs toward decreasing position)."""
    return raw - 65536 if raw >= 32768 else raw


def _raw_to_deg(raw: int) -> float:
    """Convert raw steps (0-16383) to absolute degrees. Manual: 4096 = 90°."""
    return raw * _DEG_PER_STEP


# Register addresses — manual section 2-1 Address Table, pages 12-15
class _R:
    # Status (Read Only) — manual section 2-3
    POSITION         = 0x0C  # Current position, 0-16383, 4096=90° (p.16)
    VELOCITY         = 0x0E  # Current velocity, pos/100ms (p.17)
    TORQUE           = 0x10  # Motor PWM duty, 0-4095=100% (p.17)
    VOLTAGE          = 0x12  # Supply voltage, 100=1.00V (p.17)
    MCU_TEMPER       = 0x14  # MCU temperature, 1=1°C (p.18)
    CURRENT          = 0x16  # Current draw, 1=1mA (p.20)
    TURN_COUNT       = 0x18  # Accumulated turn count (p.18)
    EMERGENCY_STOP   = 0x48  # Error flags bitfield (p.16)
    PRODUCT_NO       = 0x74  # Product number (p.19)
    VERSION          = 0xFC  # Firmware version (p.19)

    # Action (Read/Write) — manual section 2-5
    POSITION_NEW     = 0x1E  # Target position, 0-16383, 4096=90° (p.21)

    # Communication — manual section 2-6
    ID               = 0x32  # Servo ID, 0-254 (p.22)
    BAUDRATE         = 0x38  # Baud rate setting (p.22)
    SAMPLE_POINT     = 0x40  # 0=50%, 1=87.5% (p.23)
    CAN_MODE         = 0x6A  # 0=2.0A, 1=2.0B, 2=DroneCAN (p.24)

    # Mode — manual section 2-7
    RUN_MODE         = 0x44  # 0=multi-turn, 1=servo, 2=CR, 3=speed (p.25)

    # Option — manual section 2-8
    DEADBAND         = 0x4E  # Position deadband, 0-4095 (p.30)
    VELOCITY_MAX     = 0x54  # Max velocity, 0-4095 (p.32)
    TORQUE_MAX       = 0x56  # Max torque, 0-4095=100%, default 4095 (p.32)
    SPEED_UP         = 0xDC  # Acceleration time ms (p.31)
    SPEED_DN         = 0xDE  # Deceleration time ms (p.31)

    # Position limits — manual section 2-7.4-2-7.6
    # "If POSITION_NEW > POS_MAX_LIMIT, it does not move" (p.26)
    # "If POSITION_NEW < POS_MIN_LIMIT, it does not move" (p.26)
    POS_MAX_LIMIT    = 0xB0  # Max limit (p.26)
    POS_MIN_LIMIT    = 0xB2  # Min limit (p.26)
    POS_MID          = 0xC2  # Center, default 8192=180° (p.26)

    # System — manual section 2-8.7, 2-11
    POWER_CONFIG     = 0x46  # Bit 0 = SW reset (p.29)
    FACTORY_DEFAULT  = 0x6E  # 3855=factory, 0xFFFF=reload saved (p.36)
    CONFIG_SAVE      = 0x70  # Write 0xFFFF to save (p.36)


# New Packet Format command bytes — manual section 1-4, pages 5-7
_CMD_READ            = 0x72  # 'r' — read request
_CMD_READ_MULTI      = 0x52  # 'R' — read 2 registers
_CMD_WRITE           = 0x77  # 'w' — write, no response
_CMD_WRITE_READBACK  = 0x78  # 'x' — write + readback response
_CMD_RESPONSE        = 0x76  # 'v' — read response
_CMD_RESPONSE_MULTI  = 0x56  # 'V' — multi-read response

# CAN arbitration ID — servo listens/responds on 0x000 in CAN 2.0A mode
_CAN_ARB_ID = 0x000


# =============================================================================
# Telemetry data class
# =============================================================================

@dataclass
class Telemetry:
    """Servo telemetry snapshot."""
    servo_id: int
    position_deg: float       # Absolute position (0-360°)
    position_relative: float  # Offset from center (-180 to +180°)
    position_raw: int         # Raw position (0-16383)
    velocity: int             # Velocity in pos/100ms
    torque_percent: float     # Motor PWM duty (0-100%)
    voltage: float            # Supply voltage in volts
    temperature_c: int        # MCU temperature in °C
    current_ma: int           # Current draw in mA
    errors: list              # Active error strings

    def __str__(self) -> str:
        err = ", ".join(self.errors) if self.errors else "none"
        return (
            f"Servo {self.servo_id}: "
            f"pos={self.position_deg:.1f}° ({self.position_relative:+.1f}°) "
            f"vel={self.velocity} "
            f"torque={self.torque_percent:.1f}% "
            f"V={self.voltage:.2f}V "
            f"T={self.temperature_c}°C "
            f"I={self.current_ma}mA "
            f"err=[{err}]"
        )


# =============================================================================
# Low-level protocol — verified against manual section 1-4 (New Packet Format)
# =============================================================================

class _Protocol:
    """Thread-safe Hitec CAN register read/write with retry logic."""

    def __init__(self, bus: can.Bus, timeout: float = 0.3, retries: int = 2):
        self._bus = bus
        self._timeout = timeout
        self._retries = retries
        self._lock = threading.Lock()

    def _flush(self):
        """Drain any pending RX frames."""
        while self._bus.recv(timeout=0.005):
            pass

    def read(self, servo_id: int, addr: int) -> Optional[int]:
        """
        Read a 16-bit register.
        Manual 1-4 p.6: Send [0x72, ID, Addr], expect [0x76, ID, Addr, Lo, Hi].
        """
        for _ in range(self._retries + 1):
            with self._lock:
                self._flush()
                msg = can.Message(
                    arbitration_id=_CAN_ARB_ID,
                    data=bytes([_CMD_READ, servo_id, addr]),
                    is_extended_id=False,
                )
                try:
                    self._bus.send(msg)
                except can.CanError:
                    continue

                deadline = time.monotonic() + self._timeout
                while time.monotonic() < deadline:
                    resp = self._bus.recv(timeout=0.02)
                    if resp is None:
                        continue
                    d = resp.data
                    if (len(d) >= 5
                            and d[0] == _CMD_RESPONSE
                            and d[2] == addr):
                        return d[3] | (d[4] << 8)
        return None

    def read2(self, servo_id: int, addr_a: int, addr_b: int) -> Optional[tuple]:
        """
        Read two registers in one transaction.
        Manual 1-4 p.6: Send [0x52, ID, AddrA, AddrB],
        expect [0x56, ID, AddrA, LoA, HiA, AddrB, LoB, HiB].
        """
        for _ in range(self._retries + 1):
            with self._lock:
                self._flush()
                msg = can.Message(
                    arbitration_id=_CAN_ARB_ID,
                    data=bytes([_CMD_READ_MULTI, servo_id, addr_a, addr_b]),
                    is_extended_id=False,
                )
                try:
                    self._bus.send(msg)
                except can.CanError:
                    continue

                deadline = time.monotonic() + self._timeout
                while time.monotonic() < deadline:
                    resp = self._bus.recv(timeout=0.02)
                    if resp is None:
                        continue
                    d = resp.data
                    if (len(d) >= 8
                            and d[0] == _CMD_RESPONSE_MULTI
                            and d[2] == addr_a
                            and d[5] == addr_b):
                        va = d[3] | (d[4] << 8)
                        vb = d[6] | (d[7] << 8)
                        return (va, vb)
        return None

    def write(self, servo_id: int, addr: int, value: int,
              readback: bool = True) -> Optional[int]:
        """
        Write a 16-bit register.
        Manual 1-4 p.6: [0x78, ID, Addr, Lo, Hi] for write+readback,
                         [0x77, ID, Addr, Lo, Hi] for write-only.
        """
        value = value & 0xFFFF
        lo = value & 0xFF
        hi = (value >> 8) & 0xFF
        cmd = _CMD_WRITE_READBACK if readback else _CMD_WRITE

        for _ in range(self._retries + 1):
            with self._lock:
                self._flush()
                msg = can.Message(
                    arbitration_id=_CAN_ARB_ID,
                    data=bytes([cmd, servo_id, addr, lo, hi]),
                    is_extended_id=False,
                )
                try:
                    self._bus.send(msg)
                except can.CanError:
                    continue

                if not readback:
                    return value

                deadline = time.monotonic() + self._timeout
                while time.monotonic() < deadline:
                    resp = self._bus.recv(timeout=0.02)
                    if resp is None:
                        continue
                    d = resp.data
                    if (len(d) >= 5
                            and d[0] == _CMD_RESPONSE
                            and d[2] == addr):
                        return d[3] | (d[4] << 8)
        return None

    def write_fire(self, servo_id: int, addr: int, value: int):
        """Write without waiting for response. For reset/emergency."""
        value = value & 0xFFFF
        with self._lock:
            msg = can.Message(
                arbitration_id=_CAN_ARB_ID,
                data=bytes([_CMD_WRITE, servo_id, addr,
                            value & 0xFF, (value >> 8) & 0xFF]),
                is_extended_id=False,
            )
            try:
                self._bus.send(msg)
            except can.CanError:
                pass


# =============================================================================
# Single Servo
# =============================================================================

class HitecServo:
    """
    Controls one Hitec CAN servo.

    POSITION MODES:
        Absolute: move(degrees) — 0 to 360°, where 180° is center.
        Relative: move_relative(degrees) — offset from center (e.g. ±150°).

    POSITION LIMITS:
        The servo SILENTLY IGNORES any move command outside its limits.
        MDB961WP defaults to ±60° (120°-240°). Max programmable ±150° (30°-330°).
        Call set_limits(30, 330) or set_limits_relative(150) to unlock full range.
        Requires save() + power cycle to persist.
    """

    def __init__(self, proto: _Protocol, servo_id: int):
        self._p = proto
        self.id = servo_id

    # --- Motion ---------------------------------------------------------------

    def move(self, degrees: float) -> bool:
        """
        Move to absolute position in degrees (0-360°). 180° is center.

        A target outside the servo's position limits is CLAMPED to the limit,
        not rejected: the servo drives to the limit and stops there, acks the
        command, and raises no error flag (verified on MDB961WP hardware). Check
        the target against get_limits() first if that motion would be unsafe.

        Returns True if command was acknowledged.
        """
        result = self._p.write(self.id, _R.POSITION_NEW, _deg_to_raw(degrees))
        return result is not None

    def move_relative(self, offset_deg: float) -> bool:
        """
        Move relative to center. 0 = center (180°).
        -150 = 30° absolute, +150 = 330° absolute.
        SILENTLY IGNORED if outside position limits.
        """
        return self.move(_CENTER_DEG + offset_deg)

    def move_raw(self, raw: int) -> bool:
        """Move to raw position (0-16383). 4096 = 90°."""
        result = self._p.write(self.id, _R.POSITION_NEW,
                               max(0, min(_MAX_RAW, raw)))
        return result is not None

    def center(self) -> bool:
        """Move to center (180° absolute, 0° relative)."""
        return self.move(_CENTER_DEG)

    # --- Telemetry ------------------------------------------------------------

    def position(self) -> Optional[float]:
        """Read current position in absolute degrees (0-360°)."""
        raw = self._p.read(self.id, _R.POSITION)
        return _raw_to_deg(raw) if raw is not None else None

    def position_relative(self) -> Optional[float]:
        """Read current position as offset from center in degrees."""
        raw = self._p.read(self.id, _R.POSITION)
        return (_raw_to_deg(raw) - _CENTER_DEG) if raw is not None else None

    def position_raw(self) -> Optional[int]:
        """Read current raw position (0-16383)."""
        return self._p.read(self.id, _R.POSITION)

    def velocity(self) -> Optional[int]:
        """Read velocity (pos/100ms). Manual 2-3.3. Negative = decreasing position."""
        raw = self._p.read(self.id, _R.VELOCITY)
        return _s16(raw) if raw is not None else None

    def torque(self) -> Optional[float]:
        """Read torque as percentage (0-100%). Manual 2-3.4."""
        raw = self._p.read(self.id, _R.TORQUE)
        return (raw / 4095.0 * 100.0) if raw is not None else None

    def voltage(self) -> Optional[float]:
        """Read supply voltage in volts. Manual 2-3.5: 100 = 1.00V."""
        raw = self._p.read(self.id, _R.VOLTAGE)
        return (raw / 100.0) if raw is not None else None

    def temperature(self) -> Optional[int]:
        """Read MCU temperature in °C. Manual 2-3.6."""
        raw = self._p.read(self.id, _R.MCU_TEMPER)
        return _s16(raw) if raw is not None else None

    def current(self) -> Optional[int]:
        """Read current draw in mA. Manual 2-4.1. Reads slightly negative at idle."""
        raw = self._p.read(self.id, _R.CURRENT)
        return _s16(raw) if raw is not None else None

    def errors(self) -> List[str]:
        """Read error flags. Manual 2-3.1 p.16. Returns list of error strings."""
        raw = self._p.read(self.id, _R.EMERGENCY_STOP)
        if raw is None:
            return ["READ_FAILED"]
        errs = []
        if raw & (1 << 8):  errs.append("POS_MIN")
        if raw & (1 << 9):  errs.append("POS_MAX")
        if raw & (1 << 10): errs.append("TEMP_UNDER")
        if raw & (1 << 11): errs.append("TEMP_OVER")
        if raw & (1 << 13): errs.append("VOLT_UNDER")
        if raw & (1 << 14): errs.append("VOLT_OVER")
        return errs

    def telemetry(self) -> Optional[Telemetry]:
        """Read all telemetry using multi-read for efficiency."""
        pv = self._p.read2(self.id, _R.POSITION, _R.VELOCITY)
        if pv is None:
            return None
        pos_raw, vel = pv

        tv = self._p.read2(self.id, _R.TORQUE, _R.VOLTAGE)
        if tv is None:
            return None
        torque_raw, volt_raw = tv

        tc = self._p.read2(self.id, _R.MCU_TEMPER, _R.CURRENT)
        temp_raw, cur_raw = (0, 0) if tc is None else tc

        return Telemetry(
            servo_id=self.id,
            position_deg=_raw_to_deg(pos_raw),
            position_relative=_raw_to_deg(pos_raw) - _CENTER_DEG,
            position_raw=pos_raw,
            velocity=_s16(vel),
            torque_percent=torque_raw / 4095.0 * 100.0,
            voltage=volt_raw / 100.0,
            temperature_c=_s16(temp_raw),
            current_ma=_s16(cur_raw),
            errors=self.errors(),
        )

    # --- Torque Limiting ------------------------------------------------------

    def set_torque_limit(self, percent: float) -> bool:
        """
        Set max torque (0-100%). Manual 2-8.17: REG_TORQUE_MAX, 4095=100%.
        Takes effect immediately. save() to persist.

        WARNING: No internal overheating protection. Stalling >5-10s can damage servo.
        """
        raw = round(max(0.0, min(100.0, percent)) / 100.0 * 4095)
        result = self._p.write(self.id, _R.TORQUE_MAX, raw)
        return result is not None

    def get_torque_limit(self) -> Optional[float]:
        """Read current torque limit (0-100%)."""
        raw = self._p.read(self.id, _R.TORQUE_MAX)
        return (raw / 4095.0 * 100.0) if raw is not None else None

    # --- Speed Control --------------------------------------------------------

    def set_max_velocity(self, value: int) -> bool:
        """Set max velocity (0-4095). 0 = no limit. Manual 2-8.16."""
        return self._p.write(self.id, _R.VELOCITY_MAX,
                             max(0, min(4095, value))) is not None

    def set_acceleration(self, ms: int) -> bool:
        """Set acceleration ramp in ms. 0 = instant. Manual 2-8.12."""
        return self._p.write(self.id, _R.SPEED_UP, max(0, ms)) is not None

    def set_deceleration(self, ms: int) -> bool:
        """Set deceleration ramp in ms. 0 = instant. Manual 2-8.13."""
        return self._p.write(self.id, _R.SPEED_DN, max(0, ms)) is not None

    # --- Position Limits ------------------------------------------------------

    def set_limits(self, min_deg: float, max_deg: float) -> bool:
        """
        Set position limits in absolute degrees.
        For ±150°: set_limits(30, 330)
        For ±60°:  set_limits(120, 240)

        Manual 2-7.4/5/6 p.26:
        - Commands outside limits are SILENTLY IGNORED (no clamp, no move).
        - POS_MID must be set to center of min/max.
        Requires save() + power cycle to persist.
        """
        raw_min = _deg_to_raw(min_deg)
        raw_max = _deg_to_raw(max_deg)
        raw_mid = (raw_min + raw_max) // 2
        r1 = self._p.write(self.id, _R.POS_MIN_LIMIT, raw_min)
        r2 = self._p.write(self.id, _R.POS_MAX_LIMIT, raw_max)
        r3 = self._p.write(self.id, _R.POS_MID, raw_mid)
        return all(r is not None for r in [r1, r2, r3])

    def set_limits_relative(self, half_range_deg: float) -> bool:
        """
        Set symmetric limits around center.
        set_limits_relative(150) → 30° to 330° (±150°)
        set_limits_relative(60)  → 120° to 240° (±60°)
        """
        return self.set_limits(_CENTER_DEG - half_range_deg,
                               _CENTER_DEG + half_range_deg)

    def get_limits(self) -> Optional[Tuple[float, float]]:
        """Read current position limits as (min_deg, max_deg)."""
        result = self._p.read2(self.id, _R.POS_MIN_LIMIT, _R.POS_MAX_LIMIT)
        if result is None:
            return None
        return (_raw_to_deg(result[0]), _raw_to_deg(result[1]))

    def set_deadband(self, value: int) -> bool:
        """Set position deadband (0-4095, ≤20 recommended). Manual 2-8.8."""
        return self._p.write(self.id, _R.DEADBAND,
                             max(0, min(4095, value))) is not None

    # --- Configuration --------------------------------------------------------

    def set_id(self, new_id: int) -> bool:
        """Change servo ID. Needs save() + power cycle. One servo at a time!"""
        return self._p.write(self.id, _R.ID, new_id) is not None

    def firmware_version(self) -> Optional[int]:
        """Read firmware version register."""
        return self._p.read(self.id, _R.VERSION)

    def product_number(self) -> Optional[int]:
        """Read product number."""
        return self._p.read(self.id, _R.PRODUCT_NO)

    def save(self) -> bool:
        """Save settings to flash. Manual 1-7: write 0xFFFF, wait 1s."""
        result = self._p.write(self.id, _R.CONFIG_SAVE, 0xFFFF)
        time.sleep(1.0)
        return result is not None

    def reset(self):
        """Software reset. Manual 2-8.7: POWER_CONFIG bit 0."""
        self._p.write_fire(self.id, _R.POWER_CONFIG, 0x0001)

    def save_and_reset(self):
        """Save then reset."""
        self.save()
        time.sleep(0.5)
        self.reset()

    def factory_reset(self) -> bool:
        """Restore factory defaults. WARNING: erases everything including ID."""
        return self._p.write(self.id, _R.FACTORY_DEFAULT, 3855) is not None

    def ping(self) -> bool:
        """Check if servo responds."""
        return self._p.read(self.id, _R.VERSION) is not None

    def info(self) -> Optional[str]:
        """Read servo summary: ID, firmware, limits, torque limit."""
        fw = self.firmware_version()
        if fw is None:
            return None
        limits = self.get_limits()
        tlimit = self.get_torque_limit()
        lim_str = f"{limits[0]:.0f}°-{limits[1]:.0f}°" if limits else "?"
        tlim_str = f"{tlimit:.0f}%" if tlimit else "?"
        return (f"Servo {self.id}: FW=0x{fw:04X} "
                f"limits={lim_str} torque_max={tlim_str}")

    def read_reg(self, addr: int) -> Optional[int]:
        """Read any register (debugging)."""
        return self._p.read(self.id, addr)

    def write_reg(self, addr: int, value: int) -> Optional[int]:
        """Write any register (debugging)."""
        return self._p.write(self.id, addr, value)


# =============================================================================
# Bus Manager
# =============================================================================

class HitecBus:
    """
    Manages CAN bus and multiple servos.

        with HitecBus("can0") as bus:
            s1 = bus.servo(1)
            s2 = bus.servo(2)
            s1.move(90)
            s2.move(270)
    """

    def __init__(self, interface: str = "can0", bitrate: int = 1000000,
                 timeout: float = 0.3, retries: int = 2):
        self._interface = interface
        self._bus = can.Bus(channel=interface, bustype="socketcan",
                            bitrate=bitrate)
        self._proto = _Protocol(self._bus, timeout=timeout, retries=retries)
        self._servos: Dict[int, HitecServo] = {}

    def servo(self, servo_id: int) -> HitecServo:
        """Get servo by ID. Reuses instances."""
        if servo_id not in self._servos:
            self._servos[servo_id] = HitecServo(self._proto, servo_id)
        return self._servos[servo_id]

    def broadcast(self) -> HitecServo:
        """Broadcast servo (ID 0). Commands hit ALL servos."""
        return self.servo(0)

    def scan(self, max_id: int = 20) -> List[int]:
        """Scan for responding servos."""
        found = []
        for sid in range(1, max_id + 1):
            if self._proto.read(sid, _R.VERSION) is not None:
                found.append(sid)
        return found

    def move_all(self, degrees: float):
        """Move all servos to same absolute position."""
        self._proto.write(0, _R.POSITION_NEW, _deg_to_raw(degrees),
                          readback=False)

    def emergency_stop(self):
        """Software reset ALL servos."""
        self._proto.write_fire(0, _R.POWER_CONFIG, 0x0001)

    def close(self):
        """Shut down CAN bus."""
        self._bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# =============================================================================
# Setup helper
# =============================================================================

def setup_servo(interface: str = "can0", new_id: int = 1):
    """
    Configure a new servo. Connect ONLY ONE at a time.
    Sets: ID, CAN 2.0A, 1Mbps, 50% sample point, servo mode, ±150° limits.
    Power cycle the servo after.
    """
    with HitecBus(interface) as bus:
        s = bus.broadcast()

        if not s.ping():
            print("ERROR: No servo found!")
            return False

        fw = s.firmware_version()
        print(f"Found servo — FW: 0x{fw:04X}" if fw else "Found servo")

        limits = s.get_limits()
        if limits:
            print(f"  Current limits: {limits[0]:.0f}°-{limits[1]:.0f}°")

        s.write_reg(_R.CAN_MODE, 0)        # CAN 2.0A
        s.write_reg(_R.BAUDRATE, 0)         # 1Mbps
        s.write_reg(_R.SAMPLE_POINT, 0)     # 50%
        s.write_reg(_R.RUN_MODE, 1)         # Servo mode
        s.set_limits(30, 330)               # ±150° range
        s.set_id(new_id)

        print(f"  CAN mode:      CAN 2.0A")
        print(f"  Baud rate:     1Mbps")
        print(f"  Sample point:  50%")
        print(f"  Run mode:      Servo")
        print(f"  Limits:        30°-330° (±150°)")
        print(f"  Servo ID:      {new_id}")

        print("  Saving...")
        s.save()
        print("  Resetting...")
        s.reset()
        print(f"\nDone! Power cycle the servo, then use bus.servo({new_id})")
        return True
