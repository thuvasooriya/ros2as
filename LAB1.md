# lab 1: zumo ROS interface and IMU sensor calibration

## 1. introduction

### zumo 32u4 robot overview

the zumo 32u4 robot is a complete, versatile robot controlled by an arduino-compatible atmega32u4 microcontroller with the following features:[

- **main controller:** atmega32u4 AVR microcontroller from atmel
- **motor control:** dual H-bridge drivers for robot motors
- **sensors:**
  - quadrature encoders
  - inertial sensors on main board
  - reflectance sensors on front array
  - proximity sensors on front array

### on-board IMU sensors

the zumo 32u4 includes two inertial sensor chips:

1. **st lsm303d:** combines 3-axis accelerometer + 3-axis magnetometer
2. **st l3gd20h:** 3-axis gyroscope sensor

**important note:** these sensors must be calibrated before use in any application.

### learning objectives

- connect to zumo 32u4 robot via ROS interface
- perform calibration of imu sensors (accelerometer and magnetometer)

---

## 2. theoretical background

### 2.1 ROS interface architecture

the system uses a multi-node RSO architecture where:

- **zumo_serial_node:** queries zumo sensors and publishes data as ROS messages; accepts motion commands
- **calibration nodes:** process sensor data
  - **zumo_acc_calib** - performs accelerometer calibration
  - **zumo_mag_calib** - performs magnetometer calibration
- nodes communicate through ROS topics for data exchange

### 2.2 calibration fundamentals

**definition:** calibration is the process of checking the accuracy of a measuring instrument by comparison with a standard input, and adjusting the instrument to align with that standard.

#### accelerometer calibration

uses earth's **gravitational acceleration** (9.81 m/s² downward) as the known reference.

```sh
#TODO: include calibration matrix formula and calculation scripts
```

**solution method:** use **least squares method** to compute the calibration matrix C by collecting multiple measurements at different orientations.

#### magnetometer calibration

focuses on removing **hard iron distortion** caused by:

- materials emitting magnetic fields (magnets, speakers, motors)
- creates constant offset in readings

**calibration process:**

1. rotate robot through all 3 axes while collecting data
2. record minimum and maximum values for each axis (mx, my, mz)
3. calculate offset for each axis
4. subtract offset from raw sensor output

the calibration transforms distorted ellipsoidal readings into a sphere centered at the origin.

---

## 3. laboratory procedure

### 3.1 folder structure

```
zumo/
├── zumo_calibration/
├── zumo_defs/
├── zumo_launch/
├── zumo_msgs/
└── zumo_serial/
```

### 3.3 device manager rules (optionaL, linux specific)

copy the `a-star.rules` file to ubuntu udev rules folder:

```bash
sudo cp -a ./a-star.rules /etc/udev/rules.d/
```

this defines how devices are identified and managed by the linux system.

### 3.4 build ROS workspace

navigate to your ROS workspace and build:

```bash
cd REPO_PATH
pixi run build
# colcon build --symlink-install
```

**important:** the `--symlink-install` parameter creates symbolic links instead of copying files, allowing immediate effect of changes to launch files without rebuilding.

### 3.5 zumo connection

1. connect zumo 32u4 to computer using usb a to micro-b cable
2. verify connection

```bash
ls /dev/tty*
```

check for `ttyACM*` or any `ttyUSB*`

3. set device permissions if inaccesible

```bash
sudo chmod a+rw /dev/ttyACM0
```

### 3.6 running the lab

execute the following steps:

1. start zumo ROS interface (section 4.1)
2. observe zumo sensor data (section 4.2)
3. perform sensor calibration:
   - accelerometer calibration (section 4.3)
   - magnetometer calibration (section 4.3)
4. obtain and save calibration parameters

---

## 4. ROS commands reference

### 4.1 start zumo interface

```bash
# launch serial communication with the zumo robot
# can specify the device name in justfile by editing the PORT or as an argument
just serial <optional device name>
```

### 4.2 observe sensor data

#### echo sensor data in terminal

```bash
just sensors
# ros2 topic echo /zumo/zumo_sensors
```

rotate wheels or change robot orientation to observe sensor value changes.

#### visualize with rqt

```bash
pixi run rqt
```

navigate to: **plugins → visualization → plot**

**available topics for plotting:**

| sensor type      | topics                                                                  |
| ---------------- | ----------------------------------------------------------------------- |
| **acceleration** | `/zumo/zumo_sensors/ax` `/zumo/zumo_sensors/ay` `/zumo/zumo_sensors/az` |
| **gyroscope**    | `/zumo/zumo_sensors/gx` `zumo/zumo_sensors/gy` `/zumo/zumo_sensors/gz`  |
| **magnetometer** | `/zumo/zumo_sensors/mx` `/zumo/zumo_sensors/my` `/zumo/zumo_sensors/mz` |
| **encoders**     | `/zumo/zumo_sensors/enc_left` `/zumo/zumo_sensors/enc_right`            |

### 4.3 calibration procedures

#### pre-calibration check

keep a terminal open echoing `/zumo/zumo_sensors` to ensure continuous data flow.

#### calibration

**launch accelerometer calibration nodes**

```bash
# follow on-screen instructions
# once completed, calibration matrix elements are saved at results/ for future use
just caliba
# ros2 run zumo_calibration zumo_acc_calib_node

# rotate robot in all 3 axes during calibration
# obtain minimum and maximum values for all three axes (mx, my, mz)
# press ctrl+c, values are saved to results/
just calibm
# ros2 run zumo_calibration zumo_mag_calib_node
```
