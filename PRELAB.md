## ros basics

### nodes

- an executable in ROS is called a node
- ROS provides the framework to communicate between mutiple nodes
- nodes can communicate through **topics, services or actions**
- topics work the same way as in MQTT, you can publish to or subscribe to multiple nodes
- services can be provided or used but these are one-to-one

### rqt

- rqt has many plugins a.k.a features
  -- visualization -> plot - plot numeric values from a topic
  -- introspection -> node graph - node interconnections

### rviz2

can display all kinds of data streams including 3d

### commands

```sh
# get details about all the topics advertised in a ROS system
ros2 topic list

# subscribe to a topic?
ros2 topic echo topic_name # e.g. ros2 topic echo /zumo/sensors

# running ros2 nodes
## using run
# --ros-args -p max_speed:=1.0 sets a parameter at runtime
ros2 run package_name node_name parameters_if_any # e.g.  ros2 run my_robot_pkg robot_node --ros-args -p max_speed:=1.0

## using launch
ros2 launch package_name launch_file # e.g. ros2 launch zumo_keyboard zumo_keyboard_no_calib.launch
# lauch is preferred because it allows you to set many parameters and many nodes with a script

# launch rqt
rqt

# lauch rviz2
rviz2
```

## ros workspace

- workspace is a directory containing ros packages
- the ros you install is a core workspace with a collection of basic packages and is dubbed **underlay**
- typically when working with ros we create/install many local workspaces and dub them **overlay**
- it is possible to have many different distributions of ROS and multiple corresponding overlays on top on the same system and switch between them
- workspace has a typical structure
  - src -> pkg#1 pkg#2
  - build -> various build artifacts
  - install -> pkg installations
  - log
- **colcon** is the build tool we use with ROS
- CMakeLists.txt ->
- package.xml ->

```sh
# creatig the workspace directory
# this command will create a directory at your user home folder
mkdir -p ~/ros2ws/src
cd ~/ros2ws

# initialize/build with colcon
colcon build

# sourcing the overlay can be done by your preferred method
## permanently using .bashrc
echo "source $HOME/ros2ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
## using activation scripts inside install folder
# TODO:

```

## writing simple pubsub

```sh
cd ~/ros2ws

# initialize ament_cmake template
ros2 pkg create cpp_pubsub --build-type ament_cmake --destination-directory src

# write/copy your files

# check ros2 dependencies are installed
rosdep install -i --from-path src --rosdistro humble -y

# build
colcon build

# run talker/pub
ros2 run cpp_pubsub talker

# NOTE: you need another terminal session
# run the listener/sub
ros2 run cpp_pubsub listener
```

## submission

1. find out where the binaries are created when colcon build command is executed. -> install
2. get screenshots of the publisher and the subscriber output windows.
3. using rqt node graph visualize the node graph and how the nodes are connected with each other; obtain a screenshot.
4. create a second publisher that publishes 𝐴sin𝜔𝑡, where 𝐴 and 𝜔 are user defined constants.
   you may use a suitable message type from ros std_msgs (https://index.ros.org/p/std_msgs/). the original publisher should not be modified.
5. create a second subscriber that listens to above 𝐴sin𝜔𝑡 topic. the original subscriber should not be modified.
6. obtain screenshots of the modified publisher and subscriber output windows.
7. using rqt,
   a. node graph: visualize the new node graph for the modified scenario.
   b. plot: visualize the 𝐴sin𝜔𝑡 topic.

submit your answer document (one member from each group, indicating the group number and group members) to moodle.
