# ros2labs

ros2 workspace for labs

| platform         | tested | notes                      |
| ---------------- | ------ | -------------------------- |
| macos (arm64)    | ✓      | macos tahoe version 26.0.1 |
| macos (x86_64)   | ?      | should work                |
| linux (x86_64)   | ?      | should work                |
| linux (arm64)    | ?      | should work                |
| windows (x86_64) | ?      | volunteers needed          |
| windows (arm64)  | ...    | windows 11 testing         |

---

## setup

### requirements

1. [pixi](https://pixi.sh/latest/installation/)
2. [just](https://github.com/casey/just#installation)
3. [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

#### unix

```bash
# install pixi
curl -fsSL https://pixi.sh/install.sh | bash
# install just, replace DEST with where you want to install
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to DEST
# git is preinstalled on unix systems
```

#### windows

```ps1
# install pixi
powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
# install git
winget install -e --id Git.Git
# install just
winget install --id Casey.Just --exact
```

### installation

```bash
# go to cloned directory and
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
just sensors

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
