# ros2labs

ros2 workspace for autonomous systems labs

tested in macos 26

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

# interactively select which serial port to use
just select_port
# connect to zumo robot
just serial

# monitor sensor data
just mon_sensors

# run calibration procedures (lab 1)
just calib_acc
just calib_mag

# update kalman filter with calibration values
just update_kflaunch

# run kalman filter (lab 2)
just load_zumodesc
just kflaunch
just rviz_imu

# prelab pubsub examples
just pubsubtalker
just pubsublistener
just pubsubtalker_sin
just pubsublistener_sin
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
- you can edit the `PORT` variable in justfile to match your usecase
- override with: `just serial /dev/ttyUSB0`

### permissions (linux)

```bash
# grant serial port access
sudo chmod a+rw /dev/ttyACM0

# or add user to dialout group
sudo usermod -a -G dialout $USER
```

### building

- `pixi run build` uses `colcon build --symlink-install` for faster iteration
- symlink mode allows launch file changes without rebuilding
- pixi handles ros2 environment sourcing automatically

### macos build warnings

building zumo_msgs outputs:

```sh
install_name_tool: warning: changes being made to the file will invalidate the code signature in: ./ros2labs/install/zumo_msgs/lib/libzumo_msgs__rosidl_typesupport_fastrtps_c.dylib
```

similar invalidated code signature warnings for

```
libzumo_msgs__rosidl_typesupport_introspection_c.dylib
libzumo_msgs__rosidl_typesupport_c.dylib
libzumo_msgs__rosidl_generator_py.dylib
```

and that we are faking the signature using cctools-port

```sh
[cctools-port]: generating fake signature for './ros2labs/install/zumo_msgs/lib/libzumo_msgs__rosidl_typesupport_fastrtps_c.dylib'
```

i couldn't find any proper solutions for this and it works fine with the fake signatures. if anyone knows a better solution please give me a ping or submit a pr.

---

## credits

### course information

**course:** EN4594 autonomous systems
**institution:** department of electronic and telecommunication engineering, university of moratuwa
**original author:** Dr. Peshala Jayasekara

### license

original code: copyright (c) 2024 ENTC, university of moratuwa
modifications: distributed under the same license as original code

see individual source files for detailed license information.

---

## changelog

changes authored by @thuvasooriya, co-authored by claude-sonnet-4.5

see [CHANGELOG.md](CHANGELOG.md) for detailed changes and modifications.
