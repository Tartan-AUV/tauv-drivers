https://commandmasters.com/commands/ionice-linux/

# disable intentional USB buffering:
echo 0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Run with highes I/O priority:
sudo ionice -c 1 -n 2 ./main


this acheives 1000Hz!!