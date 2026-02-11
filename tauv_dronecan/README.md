
# Jetson CAN Setup for DroneCAN

## References

* NVIDIA Jetson CAN Documentation
  [https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html](https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html)

* ArduPilot MatekL431 PWM / DShot discussion
  [https://discuss.ardupilot.org/t/using-matekl431-adapters-for-pwm-and-dshot/85781](https://discuss.ardupilot.org/t/using-matekl431-adapters-for-pwm-and-dshot/85781)

* ArduPilot AP_Periph Parameters
  [https://ardupilot.org/dev/docs/AP_Periph-Parameters.html#out-blh-mask](https://ardupilot.org/dev/docs/AP_Periph-Parameters.html#out-blh-mask)

---

# 1. Enable CAN Interface on Jetson

Configure can1 using Jetson IO tool:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

Enable can1 and reboot if prompted.

---

# 2. Load CAN Kernel Drivers

## Temporary Load (until reboot)

Load required CAN modules:

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

### Bring Interface Up

DroneCAN bitrate: 1000000
Data bitrate: currently set to 1000000

```bash
sudo ip link set can1 up type can bitrate 1000000 dbitrate 1000000 berr-reporting on fd on
```

### Bring Interface Down

```bash
sudo ip link set can1 down
```

---

# 3. Test it Tool

It is intstalled in the tools folder, if you have Xquartz installed you can use `ssh -X tauv@[ip]` or use the various scripts in that folder that work without gui

```bash
cd /home/tauv/tauv-mono/tools/CanTesting
source .venv/bin/activate
dronecan-gui-tool
```

---

# 4. Make CAN Setup Persistent

## 4.1 Persistent Kernel Modules

Automatically load CAN modules at boot:

```bash
sudo sh -c 'echo "can" >> /etc/modules'
sudo sh -c 'echo "can_raw" >> /etc/modules'
sudo sh -c 'echo "mttcan" >> /etc/modules'
```

---

## 4.2 Persistent CAN Interface Configuration

Create systemd network config:

```bash
sudo nano /etc/systemd/network/80-can.network
```

Add:

```ini
[Match]
Name=can1

[CAN]
BitRate=1000000
DataBitRate=1000000
FDMode=yes
BERReporting=yes
RestartSec=100ms
```

Enable and restart:

```bash
sudo systemctl enable systemd-networkd
sudo systemctl restart systemd-networkd
```

Reboot to verify:

```bash
sudo reboot
```

---

# 5. Quick Verification Commands

Check interface:

```bash
ip -details link show can1
```

Monitor CAN traffic:

```bash
candump can1
```

