# ros2labs

ros2 workspace for autonomous systems labs

---

## setup

### requirements

requires pixi package manager and just command runner:

```bash
# install pixi
curl -fsSL https://pixi.sh/install.sh | bash

# install just
# macos
brew install just

# linux
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin

# windows (via cargo)
cargo install just

# or download from https://github.com/casey/just/releases
```

### installation

```bash
# install dependencies
pixi install

# build workspace
just build
```

---

## quick start

```bash
# list all available commands
just

# connect to zumo robot (adjust PORT in justfile if needed)
just serial

# monitor sensor data
just mon_sensors

# run calibration procedures (lab 1)
just calib_acc
just calib_mag

# update kalman filter with calibration values
just update_kflaunch

# run kalman filter (lab 2)
just l2zumodesc
just kflaunch
just rviz_imu

# prelab pubsub examples
just plab_talker
just plab_listener
just plab_sine_talker
just plab_sine_listener
```

---

## packages

- **cpp_pubsub**: basic ros2 publisher/subscriber examples
- **zumo_msgs**: custom message definitions for zumo sensors
- **zumo_serial**: serial communication node (crossplatform via boost::asio)
- **zumo_calibration**: accelerometer and magnetometer calibration tools
- **zumo_imu_kf**: imu kalman filter implementation
- **zumo_description**: urdf robot description
- **zumo_launch**: launch files for zumo nodes
- **zumo_defs**: shared definitions and constants

---

## documentation

- [PRELAB.md](PRELAB.md) - ros basics and pubsub implementation
- [LAB1.md](LAB1.md) - zumo ros interface and imu sensor calibration
- [LAB2.md](LAB2.md) - zumo orientation estimation using kalman filter

---

## notes

### serial port configuration

- default serial port is `/dev/ttyACM0` (linux)
- macos uses `/dev/cu.*` or `/dev/tty.*` for serial devices
- edit the `PORT` variable in justfile to match your system
- override with: `just serial /dev/ttyUSB0`

### permissions (linux)

```bash
# grant serial port access
sudo chmod a+rw /dev/ttyACM0

# or add user to dialout group
sudo usermod -a -G dialout $USER
```

### building

- uses `colcon build --symlink-install` for faster iteration
- symlink mode allows launch file changes without rebuilding
- pixi handles ros2 environment sourcing automatically

---

## credits

### course information

**course:** EN4594 autonomous systems  
**institution:** department of electronic and telecommunication engineering, university of moratuwa  
**original author:** peshala jayasekara  
**modifications:** cross-platform support, automation scripts, build system improvements

### license

original code: copyright (c) 2024 ENTC, university of moratuwa  
modifications: distributed under the same license as original code

see individual source files for detailed license information.

---

## changelog

see [CHANGELOG.md](CHANGELOG.md) for detailed changes and modifications.
