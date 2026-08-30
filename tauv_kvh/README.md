# tauv_kvh

ROS 2 driver for the **KVH 1775** fiber-optic gyro (FOG) IMU, running in
**Format B at 1000 Hz**. The package wraps a standalone C++ serial driver in a
composable `rclcpp` node that publishes gyro and (gravity-removed) accelerometer
data for the state estimator.

## Layout

| Path | What it is |
|---|---|
| `src/kvh_node.cpp` | ROS 2 component node (`tauv_kvh::KvhNode`) — the ROS wrapper. |
| `KVH_1775_IMU_driver/` | Standalone, ROS-independent C++ driver: serial I/O, Format-B datagram parsing, CRC, and a `main.cpp` test harness. |
| `launch/imu.launch.py` | Convenience launch file. |
| `KVH_1775_IMU_driver/1775_imu.pdf` | KVH datasheet / protocol reference. |

The `KVH_1775_IMU_driver/` directory builds independently (see its
`CMakeLists.txt`) and can be run on its own via `main.cpp` for bring-up/testing.

## Node: `kvh_node` (`tauv_kvh::KvhNode`)

Spawns a dedicated read thread that pulls Format-B packets off the serial port,
validates each packet's checksum and sensor-status flags, and publishes. If the
port isn't open at startup it keeps retrying inside the read loop.

### Gravity removal

The accelerometer output has gravity projected out before publishing. The node:

1. Subscribes to filtered odometry (`odom_topic`) to track the vehicle's
   `odom → base_link` orientation.
2. Looks up the **static** `base_link → imu` transform once (cached to avoid TF
   lookups in the 1000 Hz loop).
3. Rotates the world gravity vector `(0, 0, 9.79615)` into the IMU frame and
   subtracts it from the raw measurement.

Until the static TF is available, raw acceleration (gravity included) is
published and a throttled warning is logged.

### Published topics

Each axis is published on its own topic/frame so the estimator can fuse them
independently. Accelerometer messages set the two off-axis covariances to `1e9`
so only the axis of interest is fused.

| Topic | Type | Payload |
|---|---|---|
| `os/sensors/fog_kvh/gyro` | `sensor_msgs/Imu` | Angular velocity (x, y, z). |
| `os/sensors/fog_kvh/accel_x` | `sensor_msgs/Imu` | Gravity-removed linear accel, X. |
| `os/sensors/fog_kvh/accel_y` | `sensor_msgs/Imu` | Gravity-removed linear accel, Y. |
| `os/sensors/fog_kvh/accel_z` | `sensor_msgs/Imu` | Gravity-removed linear accel, Z. |

### Subscribed topics

| Topic | Type | Purpose |
|---|---|---|
| `odom_topic` (default `/odometry/filtered`) | `nav_msgs/Odometry` | Vehicle orientation for gravity projection. |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyTHS1` | Serial device for the KVH 1775. |
| `odom_topic` | `/odometry/filtered` | Odometry source used for gravity removal. |
| `base_frame_id` | `os/base_link` | Base frame for the static IMU transform. |
| `frame_id_gyro` | `imu_link_fog_gyro` | Frame stamped on gyro messages (also the IMU-side frame for the gravity TF). |
| `frame_id_accel_x` / `_y` / `_z` | `imu_link_fog_accel_{x,y,z}` | Frames stamped on the per-axis accel messages. |
| `gyro_noise_density` | `1.45e-5` | Gyro noise density (rad/s/√Hz). Variance = density² × `bandwidth_hz`. |
| `accel_noise_density` | `4.9e-4` | Accel noise density (m/s²/√Hz). Variance = density² × `bandwidth_hz`. |
| `bandwidth_hz` | `50.0` | Measurement bandwidth used to turn noise density into variance. |

The default noise densities come from the KVH 1775 datasheet
(ARW 0.05 °/√hr, accel ~50 µg/√Hz).

> **Note:** message timestamps are currently stamped with ROS receive time
> (`computeTimestamp()` ignores the on-sensor microsecond counter). Revisit this
> if you need hardware-clock timing.

## Running

Via launch:

```bash
ros2 launch tauv_kvh imu.launch.py port:=/dev/kvh1775
```

Standalone driver test (no ROS), from `KVH_1775_IMU_driver/`:

```bash
# disable USB latency buffering for full 1000 Hz throughput
echo 0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# run at highest I/O priority
sudo ionice -c 1 -n 2 ./main
```

## Bring-up & debugging

### One-time IMU configuration (`imu_config.txt`)

Before the driver will work, the IMU itself must be configured **once** over a
serial terminal — these settings are stored in the device's non-volatile memory
and persist across power cycles. `KVH_1775_IMU_driver/imu_config.txt` lists the
exact `=cmd` sequence; the important ones are:

| Command | Sets | Why the driver needs it |
|---|---|---|
| `=outputfmt,B` | Format B (40-byte packets w/ µs timestamp) | The parser only recognizes the Format B header `0xFE81FF56`. |
| `=dr,1000` | 1000 Hz output rate | Target sample rate. |
| `=angunits,RAD` + `=angfmt,RATE` | Gyro in rad/s | The driver passes gyro through **unscaled**, so the device must already output rad/s (not deg, not delta-angle). |
| `=linunits,METERS` + `=linfmt,ACCEL` | Accel in g→m/s² | The driver multiplies by `9.80665`, so the device must output acceleration (not delta-velocity). |
| `=baud,921600` | Baud rate | Must match `BAUD_RATE` in `constants.h`. |

If these don't match, packets either won't parse (wrong header) or values come
out in the wrong units/scale.

### `debug.py` — raw serial sniffer

`KVH_1775_IMU_driver/debug.py` is a tiny standalone tool (no ROS, no build) for
verifying the IMU is actually streaming and checking the packet rate. It opens
the serial port directly and prints each chunk of bytes with the millisecond gap
since the previous read:

```bash
python3 KVH_1775_IMU_driver/debug.py
```

Configure it by editing the two parameters at the top of the file:

- **Port** — the `serial.Serial("/dev/ttyTHS1", ...)` path. Set this to wherever
  the IMU enumerates (`/dev/ttyTHS1` on the Jetson UART, `/dev/ttyUSB0` over the
  FTDI USB adapter, or the `/dev/kvh1775` udev symlink).
- **Baud** — `921600`, must match the IMU's configured `=baud` and `constants.h`.

Use it for bring-up: if you see byte chunks arriving with small, steady gaps the
link and IMU config are good; if the inter-read gaps are large/lumpy, the
FTDI latency timer is still set too high (see below). It reads raw bytes only —
it does **not** decode datagrams (that's the C++ `main` test harness).

## Serial / performance notes

- Hitting a sustained 1000 Hz requires disabling the USB-serial latency timer
  (see above) — otherwise packets are batched and the effective rate drops.
- The node pins reads to a separate thread so ROS callbacks never stall the
  serial loop.
