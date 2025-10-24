# ros2as

ros2 workspace for autonomous systems labs

## setup

requires pixi:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

build workspace:

```bash
pixi run colcon build
```

## packages

- **cpp_pubsub**: basic ros2 publisher/subscriber examples
- **zumo_msgs**: custom message definitions for zumo sensors
- **zumo_serial**: serial communication node (crossplatform via boost::asio)
- **zumo_calibration**: accelerometer and magnetometer calibration tools
- **zumo_imu_kf**: imu kalman filter implementation
- **zumo_description**: urdf robot description
- **zumo_launch**: launch files for zumo nodes
- **zumo_defs**: shared definitions and constants

## notes

- default serial port is `/dev/ttyACM0` (linux). override with `zumo_serial_port` parameter
- macos uses `/dev/cu.*` or `/dev/tty.*` for serial devices
