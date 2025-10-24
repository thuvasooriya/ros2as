PORT := "/dev/ttyACM0"

default:
    just --list

format:
    find src -type f \( -name "*.h" -o -name "*.c" -o -name "*.hpp" -o -name "*.cpp" \) -exec clang-format -i {} +

serial port=PORT:
    pixi run ros2 run zumo_serial zumo_serial_node --ros-args -p zumo_serial_port:={{ port }}
