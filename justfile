PORT := "/dev/tty.usbmodem1101"

# help
default:
    just --list

# build the workspace
build:
    pixi run build

# format all cpp files in src/
[unix]
format:
    find src -type f \( -name "*.h" -o -name "*.c" -o -name "*.hpp" -o -name "*.cpp" \) -exec clang-format -i {} +

# select serial port interactively
select_port:
    pixi run scripts/select_serial_port.py

# connect to zumo serial port
serial port=PORT:
    pixi run ros2 run zumo_serial zumo_serial_node --ros-args -r __ns:=/zumo -p zumo_serial_port:={{ port }}

# monitor sensors topic
sensors:
    pixi run sensors

# visualize with rqt
rqt:
    pixi run rqt

# list all the topics
list_topics:
    pixi run ros2 topic list

# start calibration procedure for accelerometer
calib_acc:
    pixi run ros2 run zumo_calibration zumo_acc_calib_node

# start calibration procedure for magnetometer
calib_mag:
    pixi run ros2 run zumo_calibration zumo_mag_calib_node

# edit kalman filter launch file with calibration values in results/ folder
update_kflaunch:
    pixi run scripts/update_imu_calib.py

# upload zumo robot description for visualization
load_zumodesc:
    pixi run ros2 launch zumo_launch zumo_startup.launch.py

# launch kalman filter node
kflaunch:
    pixi run ros2 launch zumo_launch zumo_imu_kf.launch

# launch rviz with zumo_imu profile
rviz_imu:
    pixi run ros2 run rviz2 rviz2 -d ./.rviz2/zumo_imu.rviz

# run turtlesim tutorial
turtlesim:
    pixi run ros2 run turtlesim turtlesim_node

# run turtlesim teleop key
turtleteleop:
    pixi run ros2 run turtlesim turtle_teleop_key

# run pubsub talker
pubsubtalker:
    pixi run ros2 run cpp_pubsub talker

# run pubsub listener
pubsublistener:
    pixi run ros2 run cpp_pubsub listener

# run sine talker
pubsubtalker_sin:
    pixi run ros2 run cpp_pubsub sine_talker

# run sine listener
pubsublistener_sin:
    pixi run ros2 run cpp_pubsub sine_listener
