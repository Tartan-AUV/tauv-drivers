# tauv_kvh

ROS 2 driver for the KVH 1775 FOG IMU. The vehicle target is **500 Hz**, using
40-byte Format B packets, gyro rates in radians/second and acceleration in g.
The driver converts acceleration to m/s² and preserves the existing gravity
removal and four-topic interface.

## Running and device configuration

```bash
ros2 launch tauv_kvh imu.launch.py port:=/dev/kvh1775
```

The host defaults to **460800 baud**, matching the configured vehicle IMU at
500 Hz. The launch command above uses that default. Use `baud_rate:=921600`
only when connecting to a device configured for 921600 baud. Device configuration
instructions are in [imu_config.txt](imu_config.txt).

The driver never changes the device baud, output rate, format, units, axes or
filters. Check command replies and query actual settings. KVH specifies a
maximum of 500 Hz at 460800 baud and 1000 Hz at 921600. A lower baud may improve
the observed connection, but software tests cannot establish that for the vehicle.
Close serial terminals and sniffers before starting the driver.

## Reception and publishing

A dedicated reception thread reads up to 2048 bytes at a time into a 4096-byte
buffer. The parser drains complete buffered packets before requesting another
read. It validates the header and CRC, searches for the full header again after
corruption, and never overwrites unread bytes in the serial buffer.

The serial port is explicitly raw 8-N-1, with software/hardware flow control
disabled. Timeouts, EINTR and EAGAIN do not cause reconnects. Hard I/O failures
close the port and clear partial packets; failed opens are retried with a 100 ms
backoff. Applying port attributes does not flush queued input.

A separate thread does gravity removal and ROS publication. A preallocated queue
holds up to `sample_queue_capacity` samples (default 1000, or two seconds at
500 Hz). If publication cannot keep up, the oldest queued sample is evicted and
counted. This bounds memory and queue age; it cannot make a persistently slow
publisher lossless. Shutdown discards remaining queued samples.

The driver does not set CPU affinity or real-time scheduling. Separating the two
threads reduces the time serial reception spends waiting on publication; any
Jetson scheduling changes need measurements on the target.

## Topics, validity and gravity

All topics use `sensor_msgs/Imu`:

| Topic | Published data | Frame parameter |
|---|---|---|
| `os/sensors/fog_kvh/gyro` | All three angular rates | `frame_id_gyro` |
| `os/sensors/fog_kvh/accel_x` | X acceleration | `frame_id_accel_x` |
| `os/sensors/fog_kvh/accel_y` | Y acceleration | `frame_id_accel_y` |
| `os/sensors/fog_kvh/accel_z` | Z acceleration | `frame_id_accel_z` |

A bad CRC rejects the complete packet. A failed accelerometer status bit rejects
only that accelerometer topic; valid gyro data still publishes. The gyro topic
requires all three gyro status bits. Non-finite values are rejected for the
relevant sensor. Status bits describe physical sensors, so these rules assume
the IMU AXES configuration is identity and frames correspond to those sensors.

Gravity projection uses the latest orientation from `odom_topic` (default
`/odometry/filtered`) and a cached transform from `base_frame_id` (default
`os/base_link`) to `frame_id_gyro`. Until that transform is available, acceleration
still includes gravity, as in the previous driver. The latest odometry is not
interpolated to sample time. Verify those existing assumptions in the estimator,
including whether it also removes gravity.

The old launch argument `frame_id` did not control the node's frame parameters;
use the four explicit frame arguments above. Defaults are `imu_link_fog_gyro`
and `imu_link_fog_accel_{x,y,z}`. `publisher_depth` controls ROS history per topic
(default 20); reliability is still reliable. Subscriber and recorder queue depths
are independent. Covariance is density² × `bandwidth_hz`; the node default is
50 Hz and the launch default remains 500 Hz to preserve previous launch behavior.
Set it explicitly to match the device's effective filtering when tuning the EKF.

## Timestamps

Samples use ROS receive time, captured in the reception thread before they wait
for publication. Publication-queue delay therefore does not shift their timestamps.
Serial/OS batching can still cause receive-time jitter. The on-device timestamp
is parsed as part of Format B but is not used to stamp ROS messages.

## Diagnostics

Every five seconds the node logs cumulative counters:

- `bytes`: bytes read from the serial port.
- `packets`: CRC-valid packets, including packets with failed sensor status.
- `crc_failures`: failed CRC checks on header-aligned candidates, not on every
  shifted byte window. Header-like payload bytes can create extra candidates.
- `status_failures`: CRC-valid packets with any non-nominal sensor-status bit.
- `sequence_gaps`: missing increments modulo 128, calculated before sensor
  filtering and queue eviction. This is a lower bound after gaps of 128 or more
  packets; device resets without a reconnect can also look like sequence gaps.
- `duplicates`: successive packets with the same sequence number. They are
  counted, not automatically discarded.
- `io_errors`, `connections`: hard I/O/open/configuration failures and successful
  opens. A normal read timeout or interrupted call changes neither counter.
- `discarded_bytes`: bytes skipped while searching for a CRC-valid frame.
- `queue_drops`: samples evicted because publication fell behind reception.
- `gyro_published`, `accel_published`: publication calls (accel sums three topics).
  They do not prove receipt by the EKF or recorder.

Counter snapshots are approximate across threads. Compare them with ROS subscriber,
recorder and EKF acceptance statistics to locate losses. The standalone `main`
logger remains a bring-up tool; the ROS node provides these diagnostics.

## Building

On a ROS 2 system:

```bash
colcon build --packages-select tauv_kvh
```

The standalone serial driver can also be built without ROS:

```bash
cmake -S tauv_kvh/KVH_1775_IMU_driver -B /tmp/kvh-build
cmake --build /tmp/kvh-build
```

Hardware follow-up: compare counters over a sustained 500 Hz run, repeat under
normal vehicle load, and verify ROS delivery, timestamps and EKF rejection rates.
