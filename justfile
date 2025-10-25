# PORT := "/dev/ttyACM0"

PORT := "/dev/tty.usbmodem1101"

default:
    just --list

format:
    find src -type f \( -name "*.h" -o -name "*.c" -o -name "*.hpp" -o -name "*.cpp" \) -exec clang-format -i {} +

serial port=PORT:
    pixi run ros2 run zumo_serial zumo_serial_node --ros-args -r __ns:=/zumo -p zumo_serial_port:={{ port }}

startl1:
    pixi run ros2 launch zumo_launch zumo_serial_node.launch

sensors:
    pixi run sensors

tlist:
    pixi run ros2 topic list

caliba:
    pixi run ros2 run zumo_calibration zumo_acc_calib_node

calibm:
    pixi run ros2 run zumo_calibration zumo_mag_calib_node

update_kflaunch:
    python3 scripts/update_imu_calib.py

l2zumodesc:
    pixi run ros2 launch zumo_launch zumo_startup.launch.py

l2kflaunch:
    pixi run ros2 launch zumo_launch zumo_imu_kf.launch

l2rviz:
    # pixi run rviz2 -d ./.rviz2/zumo_imu.rviz
    pixi run ros2 run rviz2 rviz2 -d ./.rviz2/zumo_imu.rviz
