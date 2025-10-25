# changelog

all notable changes and modifications to the original assignment files.

---

## cross-platform improvements

### serial communication

- rewrote zumo_serial serial libraries with boost::asio for cross-platform support
- replaced linux-specific serial implementation
- now works on linux, macos, and windows

### build system

- added compile_commands.json generation flag in cmake
- integrated clangd support for better IDE integration
- configured proper cross-platform compiler settings

---

## calibration enhancements

### automated workflows

- implemented autosave for calibration results
- calibration data automatically saved to results/ folder
- created python script to update kalman filter launch files with calibration values
- added `just update_kflaunch` command for easy calibration integration

### launch files

- created launch files for accelerometer calibration
- created launch files for magnetometer calibration
- created launch files for serial node
- proper parameter passing and namespace configuration

---

## developer experience

### command automation

- integrated justfile for command organization
- created pixi-based task commands
- simplified workflow with memorable command names
- added help system with `just` command

### environment management

- integrated pixi for ros2 environment management
- automatic environment sourcing via pixi activation
- cross-platform dependency management
- simplified setup process

---

## python compatibility

- fixed python environment detection in zumo_msgs compilation
- resolved setuptools compatibility issues
- proper python3 integration with ros2 build system

---

## documentation

- reorganized and standardized all documentation files
- updated commands to use justfile and pixi
- added comprehensive README with quick start guide
- improved clarity and consistency across all lab documents
- added proper formatting and structure to changelog

---

## bug fixes

### format specifiers

- fixed printf format specifier warnings in zumo_mag_calib.cpp
- used PRId64 macro from cinttypes for cross-platform int64_t formatting

---

## developer experience

### serial port selection

- added interactive serial port selection script
- automatically filters relevant serial ports per platform (ttyACM/ttyUSB on linux, cu/tty on macos, COM on windows)
- updates justfile PORT variable in place
- added `just select_port` command
- added pyserial dependency to pixi.toml

---

## code quality improvements

### c++ standard upgrade

- upgraded all CMakeLists.txt files from c++14 to c++17
- improved code quality and modern c++ support across all packages

### header file cleanup

- removed `using namespace std` from all header files
- moved fstream includes from headers to cpp files
- added explicit std:: namespace qualifiers in implementation files
- improved header hygiene and reduced namespace pollution

### cross-platform defaults

- added platform-conditional serial port defaults in definitions.hpp
- windows defaults to COM1
- macos defaults to /dev/cu.usbmodem14201
- linux defaults to /dev/ttyACM0

### python environment handling

- removed hardcoded macos python paths from zumo_msgs/CMakeLists.txt (reverted - see workarounds)
- now uses pixi-managed python environment variables
- improved cross-platform python compatibility

### workarounds

**macos python path requirement:**
- rosidl_generator_py in ros2 humble requires explicit python paths on macos
- added platform-conditional python path configuration for macos in zumo_msgs/CMakeLists.txt
- uses CONDA_PREFIX environment variable provided by pixi
- paths: Python_EXECUTABLE, Python_INCLUDE_DIRS, Python_LIBRARIES
- required due to cmake FindPython module limitations with ros2 message generation on macos

### task configuration

- removed bash shell dependency from pixi.toml tasks
- standardized python script invocation to use `pixi run`
- added unix-only guard to format command in justfile

### error handling

- updated serial port error messages to reference docs/README.md
- added guidance to run `just select_port` for serial configuration
- improved user experience with actionable error messages

### project organization

- added results/ directory with .gitkeep
- added results/*.txt to .gitignore
- added compile_commands.json to .gitignore
