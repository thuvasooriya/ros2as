# prelab: ros basics and pubsub implementation

## 1. ros basics

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

---

## 2. common commands

### building the workspace

```bash
# build workspace with pixi
just build
# pixi run build
```

### running turtlesim tutorial

```bash
just turtlesim
# pixi run turtlesim
```

### topic operations

```bash
# get details about all the topics advertised in a ROS system
just list_topics
# pixi run ros2 topic list

# subscribe to a topic
pixi run ros2 topic echo /topic_name
# example: pixi run ros2 topic echo /zumo/zumo_sensors
```

### running nodes

```bash
# using run (sets parameters at runtime)
pixi run ros2 run package_name node_name --ros-args -p param_name:=value
# example: pixi run ros2 run my_robot_pkg robot_node --ros-args -p max_speed:=1.0

# using launch (preferred for multiple nodes and parameters)
pixi run ros2 launch package_name launch_file
# example: pixi run ros2 launch zumo_launch zumo_startup.launch.py
```

### visualization tools

```bash
# launch rqt
just rqt
# pixi run rqt

# launch rviz2
pixi run ros2 run rviz2 rviz2
```

---

## 3. ros workspace

## 3. ros workspace

- workspace is a directory containing ros packages
- the ros you install is a core workspace with a collection of basic packages and is dubbed **underlay**
- typically when working with ros we create/install many local workspaces and dub them **overlay**
- it is possible to have many different distributions of ROS and multiple corresponding overlays on top on the same system and switch between them
- workspace has a typical structure:
  - src → pkg#1, pkg#2, ...
  - build → various build artifacts
  - install → pkg installations
  - log → build and runtime logs
- **colcon** is the build tool we use with ROS
- CMakeLists.txt defines build configuration
- package.xml defines package metadata and dependencies

```bash
# creating the workspace directory
# this command will create a directory at your user home folder
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# initialize/build with colcon
just build
# pixi run build
# colcon build --symlink-install

# sourcing the overlay (handled automatically with pixi activation)
# if not using pixi, source manually:
source install/setup.bash
```

---

## 4. writing simple pubsub

## 4. writing simple pubsub

```bash
cd ~/ros2_ws

# initialize ament_cmake template
pixi run ros2 pkg create cpp_pubsub --build-type ament_cmake --destination-directory src

# write/copy your files

# check ros2 dependencies are installed
pixi run rosdep install -i --from-path src --rosdistro humble -y

# build
just build

# run talker/pub
just plab_talker
# pixi run plab_t2t

# run the listener/sub (in another terminal)
just plab_listener
# pixi run plab_t2l
```

---

## 5. submission tasks

1. find out where the binaries are created when colcon build command is executed → **install** folder
2. get screenshots of the publisher and the subscriber output windows
3. using rqt node graph visualize the node graph and how the nodes are connected with each other; obtain a screenshot
4. create a second publisher that publishes $$A\sin\omega t$$, where $$A$$ and $$\omega$$ are user defined constants. you may use a suitable message type from ros std_msgs (https://index.ros.org/p/std_msgs/). the original publisher should not be modified
5. create a second subscriber that listens to above $$A\sin\omega t$$ topic. the original subscriber should not be modified
6. obtain screenshots of the modified publisher and subscriber output windows
7. using rqt:
   - **node graph:** visualize the new node graph for the modified scenario
   - **plot:** visualize the $$A\sin\omega t$$ topic

### running sine pubsub

```bash
# run sine talker
just plab_sine_talker
# pixi run plab_t4

# run sine listener (in another terminal)
just plab_sine_listener
# pixi run plab_t5
```

submit your answer document (one member from each group, indicating the group number and group members) to moodle.
