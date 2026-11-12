# tauv_depth

ROS 2 driver for the Blue Robotics **Bar02** (MS5837-02BA) high-resolution
depth/pressure sensor, read over I²C. Publishes vehicle depth as an
`nav_msgs/Odometry` message for consumption by the state estimator.

## Node: `depth` (`Bar02Node`)

Reads the MS5837 barometer on a timer, converts the measured depth into a ROS-Z
position, and publishes it. If the sensor drops off the bus (bad wiring, power
glitch) the node counts failures and transparently re-initializes the sensor.

### Published topics

| Topic | Type | Notes |
|---|---|---|
| `/os/sensors/depth` | `nav_msgs/Odometry` | Only `pose.pose.position.z` is meaningful. |

Depth is measured positive-down by the sensor and negated to ROS-Z
(positive-up). The pose/twist covariance is set to `1e6` on every axis except
the Z position (row-major index 14), which carries the configured `variance` —
this tells the state estimator to fuse **only** the Z measurement.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `i2c_bus` | `7` | I²C bus number the Bar02 is wired to. |
| `freq_hz` | `10.0` | Publish/read rate in Hz. |
| `max_retries` | `5` | Consecutive read failures before the sensor is re-initialized. |
| `variance` | `0.001` | Variance (m²) placed on the Z-position covariance entry. |

The sensor is configured for **fresh water** density (`DENSITY_FRESHWATER`). For
salt-water pool/ocean work, change the `setFluidDensity()` call in
`Bar02Node::initializeSensor()` in `src/depth_node.cpp` (e.g. to
`DENSITY_SALTWATER`) and rebuild.

## Files

- `include/tauv_depth/depth_node.hpp`, `src/depth_node.cpp` — the `Bar02Node`
  ROS 2 (`rclcpp`) node (executable `depth`).
- `include/tauv_depth/ms5837.hpp`, `src/ms5837.cpp` — low-level MS5837 I²C
  register/calibration driver using Linux `/dev/i2c-*` (ported from Blue
  Robotics' reference Python library).
- `CMakeLists.txt`, `package.xml` — `ament_cmake` build configuration.

## Building

```bash
colcon build --packages-select tauv_depth
source install/setup.bash
```

If the package was previously built as `ament_python`, remove the stale
`build/tauv_depth` and `install/tauv_depth` directories before rebuilding.

## Running

```bash
ros2 run tauv_depth depth --ros-args -p i2c_bus:=7 -p freq_hz:=10.0
```

## Hardware notes

- The Bar02 is a 3.3 V I²C device; confirm the bus number with `i2cdetect -y <bus>`
  (the sensor answers at address `0x76`).
- On the Jetson, the exposed I²C buses are numbered per the carrier board — bus
  `7` is the current wiring on the vehicle.