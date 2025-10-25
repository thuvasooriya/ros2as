# lab 2: zumo orientation estimation using kalman filter

---

## 1. introduction

### MEMS IMU overview

MEMS inertial measurement units are used to estimate the orientation of an attached object.

**9-DOF IMU components:**

- **3-axis accelerometer:** measures proper acceleration (acceleration with respect to a free-falling body)
- **3-axis gyroscope:** reports angular velocity of the object in all three body axes
- **3-axis magnetometer:** measures earth's magnetic field and helps find heading direction relative to north

### on-board IMU sensors

the zumo 32u4 includes two inertial sensor chips:

1. **st lsm303d:** combines 3-axis accelerometer + 3-axis magnetometer
2. **st l3gd20h:** 3-axis gyroscope sensor

### learning objectives

- implement kalman filter assuming linear gaussian system
- estimate orientation of robot using IMU sensor data

## 2. theoretical background

### 2.1 rotation matrix

a **rotation matrix** is a transformation matrix used to perform rotations in euclidean space, representing the relative orientation between two rigid bodies.

**mathematical representation:**

$$ R = \begin{bmatrix} r*{11} & r*{12} & r*{13} \\ r*{21} & r*{22} & r*{23} \\ r*{31} & r*{32} & r\_{33} \end{bmatrix} $$

**column vector interpretation:**

- $$\begin{bmatrix} r_{11} \\ r_{21} \\ r_{31} \end{bmatrix}$$ = **x-axis** of body frame expressed in XYZ earth coordinate frame
- $$\begin{bmatrix} r_{12} \\ r_{22} \\ r_{32} \end{bmatrix}$$ = **y-axis** of body frame expressed in XYZ earth coordinate frame
- $$\begin{bmatrix} r_{13} \\ r_{23} \\ r_{33} \end{bmatrix}$$ = **z-axis** of body frame expressed in XYZ earth coordinate frame

the rotation matrix represents one coordinate frame's axes (three vectors) in another coordinate frame as its column vectors.

### 2.2 kalman filter

the **kalman filter** is a probabilistic estimation technique to estimate system state using system dynamics and sequential observations in the presence of statistical noise and other inaccuracies.

**applicability:** direct application only for **linear gaussian systems** where:

- system dynamic model is linear
- observation model is linear
- process and measurement noises are zero-mean gaussian

#### system models

**system dynamic model:**
$$ x*t = A_t x*{t-1} + B*t u*{t-1} + \varepsilon_t $$

**observation model:**
$$ z_t = C_t x_t + \delta_t $$

**where:**

- $$x_t$$ = state
- $$A_t$$ = state transition matrix
- $$B_t$$ = control input matrix
- $$u_t$$ = control input
- $$z_t$$ = observation
- $$C_t$$ = observation matrix
- $$\varepsilon_t$$ = zero-mean gaussian process noise with covariance $$R_t$$
- $$\delta_t$$ = zero-mean gaussian observation noise with covariance $$Q_t$$

#### kalman filter equations

**prediction step:**

**predicted state estimate:**
$$ \bar{\mu}_t = A_t \mu_{t-1} + B_t u_t $$

**predicted estimate covariance:**
$$ \bar{\Sigma}_t = A_t \Sigma_{t-1} A_t^T + R_t $$

**update step:**

**innovation:**
$$ \bar{z}\_t = z_t - C_t \bar{\mu}\_t $$

**innovation covariance:**
$$ S_t = C_t \bar{\Sigma}\_t C_t^T + Q_t $$

**optimal kalman gain:**
$$ K_t = \bar{\Sigma}\_t C_t^T S_t^{-1} $$

**updated state estimate:**
$$ \mu_t = \bar{\mu}\_t + K_t \bar{z}\_t $$

**updated estimate covariance:**
$$ \Sigma_t = (I - K_t C_t) \bar{\Sigma}\_t $$

### 2.3 orientation estimation using kalman filter

**key concept:** each column of the rotation matrix is considered a **state** of the system, and **three kalman filters run in parallel** for orientation estimation. we assume the three states are independent for convenience.

#### state transition

the change of rotation matrix $$R_B^E(t)$$ over time $$dt$$ is given by:

$$ R_B^E(t + dt) = \begin{bmatrix} 1 & -d\theta_z & d\theta_y \\ d\theta_z & 1 & -d\theta_x \\ -d\theta_y & d\theta_x & 1 \end{bmatrix} R_B^E(t) $$

**where:**

- $$E$$ = fixed earth frame
- $$B$$ = body frame (zumo robot)
- orientation change is represented with respect to fixed frame $$E$$

**state transition for each column vector:**

$$ x_B^E(t + dt) = \begin{bmatrix} 1 & -d\theta_z & d\theta_y \\ d\theta_z & 1 & -d\theta_x \\ -d\theta_y & d\theta_x & 1 \end{bmatrix} x_B^E(t) $$

$$ y_B^E(t + dt) = \begin{bmatrix} 1 & -d\theta_z & d\theta_y \\ d\theta_z & 1 & -d\theta_x \\ -d\theta_y & d\theta_x & 1 \end{bmatrix} y_B^E(t) $$

$$ z_B^E(t + dt) = \begin{bmatrix} 1 & -d\theta_z & d\theta_y \\ d\theta_z & 1 & -d\theta_x \\ -d\theta_y & d\theta_x & 1 \end{bmatrix} z_B^E(t) $$

where $$x_B^E(t)$$, $$y_B^E(t)$$, and $$z_B^E(t)$$ are the column vectors of $$R_B^E(t)$$.

**constructing state transition matrix using gyroscope readings:**

$$ d\theta_x = \omega_x dt $$
$$ d\theta_y = \omega_y dt $$
$$ d\theta_z = \omega_z dt $$

where $$\omega_x$$, $$\omega_y$$, $$\omega_z$$ are gyroscope angular velocity readings.

#### observations

##### 1) roll and pitch measurements from accelerometer

the accelerometer sensor generates **roll ($$\psi$$)** and **pitch ($$\theta$$)** measurements.

**rotation matrix for successive roll, pitch, yaw:**

for successive roll $$\psi$$, pitch $$\theta$$, and yaw $$\phi$$ angles (in the given order) about the fixed earth $$E$$ frame:

$$
R_B^E = \begin{bmatrix}
\cos\phi\cos\theta & -\sin\phi\cos\psi + \cos\phi\sin\theta\sin\psi & \sin\phi\sin\psi + \cos\phi\sin\theta\cos\psi \\
\sin\phi\cos\theta & \cos\phi\cos\psi + \sin\phi\sin\theta\sin\psi & -\cos\phi\sin\psi + \sin\phi\sin\theta\cos\psi \\
-\sin\theta & \cos\theta\sin\psi & \cos\theta\cos\psi
\end{bmatrix}
$$

**transforming earth's gravitational field to body frame:**

$$ g_B^{\circ} = R_E^B g_E^{\circ} $$

since $$R_E^B = (R_B^E)^{-1} = (R_B^E)^T$$, we have:

$$ \begin{bmatrix} g_x^{B} \\ g_y^{B} \\ g_z^{B} \end{bmatrix} = (R_B^E)^T \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} $$

**note:** IMU accelerometer outputs **proper acceleration** (acceleration w.r.t. a falling body), therefore we use $$\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}^T$$ instead of $$\begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix}^T$$.

**resulting gravity components:**

$$ \begin{bmatrix} g_x^{B} \\ g_y^{B} \\ g_z^{B} \end{bmatrix} = \begin{bmatrix} -\sin\theta \\ \cos\theta\sin\psi \\ \cos\theta\cos\psi \end{bmatrix} $$

**roll calculation:**
$$ \psi = \tan^{-1}\left(\frac{g_y^B}{g_z^B}\right) $$

**pitch calculation:**
$$ \theta = \tan^{-1}\left(\frac{-g_x^B}{\sqrt{(g_y^B)^2 + (g_z^B)^2}}\right) $$

##### 2) yaw measurements from magnetometer

using the **magnetometer sensor** and the roll and pitch results, yaw measurements can be obtained.

**yaw definition:** yaw $$\phi$$ is the angle of the projected x-axis of the body frame on the ground with respect to magnetic north direction in positive anticlockwise sense.

**transformation process:**

transform the magnetometer reading $$M_B^{\circ}$$ to a frame (frame-1) whose XY plane is coplanar with the earth XY plane:

$$ M_1^{\circ} = R_B^1 M_B^{\circ} $$

$$R_B^1$$ is generated by basic rotation matrices:

$$ R_B^1 = R_2^1 \cdot R_B^2 $$

thus:
$$ M_1^{\circ} = R_2^1 R_B^2 M_B^{\circ} $$

**where:**

$$ R_2^1 = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix} \quad \quad R_B^2 = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\psi & -\sin\psi \\ 0 & \sin\psi & \cos\psi \end{bmatrix} $$

**magnetometer reading:**
$$ M_B^{\circ} = \begin{bmatrix} m_x \\ m_y \\ m_z \end{bmatrix}\_B^{\circ} $$

**complete transformation:**

$$ \begin{bmatrix} m_x \\ m_y \\ m_z \end{bmatrix}\_1^{\circ} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\psi & -\sin\psi \\ 0 & \sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} m_x \\ m_y \\ m_z \end{bmatrix}\_B^{\circ} $$

**project $$M_1^{\circ}$$ onto XY plane:**

set $$m_z^1 = 0$$ to project onto the xy plane, resulting in:

$$ \begin{bmatrix} m*x \\ m_y \\ m_z \end{bmatrix}*{proj}^1 = \begin{bmatrix} m_x^B\cos\theta + m_y^B\sin\theta\sin\psi + m_z^B\sin\theta\cos\psi \\ m_y^B\cos\psi - m_z^B\sin\psi \\ 0 \end{bmatrix} $$

**yaw calculation:**

since this projected $$M_{proj}^1$$ gives north direction in frame-1, the yaw $$\phi$$ measurement is:

$$ \phi = -\tan^{-1}\left(\frac{m_y^B\cos\psi - m_z^B\sin\psi}{m_x^B\cos\theta + m_y^B\sin\theta\sin\psi + m_z^B\sin\theta\cos\psi}\right) $$

where pitch $$\theta$$ and roll $$\psi$$ angles can be replaced by the readings obtained previously.

##### 3) inclusion of observations

the obtained roll, pitch, and yaw observations are used to generate the $$R_B^E$$ matrix as given in equation (18). the columns of this matrix are compared with the predicted rotation matrix from equation (11).

### 2.4 renormalization

**purpose:** rotation matrices must comply with **orthogonality conditions**:

- column vectors should be orthogonal to each other
- column vectors should be unit vectors

**problem:** state transition gradually accumulates errors that violate orthogonality conditions. therefore, we need to normalize the column vectors to enforce orthogonality.

#### orthogonalization process

**column vectors:**
$$ x = \begin{bmatrix} r*{11} \\ r*{21} \\ r*{31} \end{bmatrix} \quad y = \begin{bmatrix} r*{12} \\ r*{22} \\ r*{32} \end{bmatrix} \quad z = \begin{bmatrix} r*{13} \\ r*{23} \\ r\_{33} \end{bmatrix} $$

**error calculation:**

orthogonality implies the dot product of column vectors is zero. any deviation indicates error:

$$ error = x \cdot y = x^T y = \begin{bmatrix} r*{11} & r*{21} & r*{31} \end{bmatrix} \begin{bmatrix} r*{12} \\ r*{22} \\ r*{32} \end{bmatrix} $$

**error correction:**

assign half the error to each column vector and approximately rotate them in opposite directions:

$$ x\_{orth} = x - \frac{error}{2} y $$

$$ y\_{orth} = y - \frac{error}{2} x $$

**third vector (cross product):**
$$ z*{orth} = x*{orth} \times y\_{orth} $$

#### normalization process

**standard normalization:**
$$ x*{norm} = \frac{x}{|x|}, \quad y*{norm} = \frac{y}{|y|}, \quad z\_{norm} = \frac{z}{|z|} $$

**fast approximation for embedded systems:**

using 1st order taylor expansion for computational efficiency:

$$ x*{norm} = \frac{1}{2}(3 - x*{orth} \cdot x*{orth})x*{orth} $$

$$ y*{norm} = \frac{1}{2}(3 - y*{orth} \cdot y*{orth})y*{orth} $$

$$ z*{norm} = \frac{1}{2}(3 - z*{orth} \cdot z*{orth})z*{orth} $$

---

## 3. laboratory procedure

### 3.1 folder structure

```
zumo/
├── zumo_calibration/
├── zumo_defs/
├── zumo_description/
├── zumo_imu_kf/
├── zumo_launch/
├── zumo_msgs/
└── zumo_serial/
```

### 3.2 configuration files

copy the `zumo_imu.rviz` file into the `.rviz2` hidden folder in your workspace root or home directory.

### 3.3 install required libraries

xacro (xml macros) is a powerful tool for defining robot models in URDF (unified robot description format) using XML macros.

**installation:**

```bash
# ubuntu/linux
sudo apt-get update && sudo apt install ros-humble-xacro

# using pixi (already included in pixi.toml)
pixi install
```

### 3.4 kalman filter implementation

implement the following functions in `zumo/zumo_imu_kf/src/zumo_imu_kf.cpp` file:

| function                         | task                             | reference section |
| -------------------------------- | -------------------------------- | ----------------- |
| `ZumoImuKF::TransformAccRawData` | roll and pitch angle calculation | section 2.3       |
| `ZumoImuKF::TransformMagRawData` | yaw angle calculation            | section 2.3       |
| `ZumoImuKF::Normalize()`         | perform renormalization          | section 2.4       |
| `ZumoImuKF::KalmanPredict()`     | kalman filter prediction step    | section 2.2       |
| `ZumoImuKF::KalmanUpdate()`      | kalman filter update step        | section 2.2       |

### 3.5 build the workspace

navigate to your ROS workspace and build:

```bash
cd REPO_PATH
pixi run build
```

**note:** if you encounter any compilation errors, check your function implementations or contact the instructor.

### 3.6 zumo connection

1. connect zumo 32u4 to computer using usb a to micro-b cable
2. verify connection

```bash
ls /dev/tty*
```

check for `ttyACM*` or any `ttyUSB*`

3. set device permissions if inaccessible

```bash
sudo chmod a+rw /dev/ttyACM0
```

### 3.7 running the lab

execute the following steps:

1. start zumo ROS interface (section 4.1)
2. observe zumo sensor data (section 4.2)
3. upload robot structure for visualization (section 4.3)
4. copy accelerometer and magnetometer calibration values from lab 1 using:
   ```bash
   just update_kflaunch
   ```
5. run the kalman filter (section 4.4)
6. observe robot orientation estimated by kalman filter (section 4.5)
7. **experiment:** comment out the `KalmanUpdate();` line in `ZumoImuKF::ZumoSensorCb` function and observe what happens in rviz2 visualization

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
# pixi run ros2 topic echo /zumo/zumo_sensors
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

### 4.3 upload zumo description

```bash
# upload robot structure for visualization purposes
just load_zumodesc
# pixi run ros2 launch zumo_launch zumo_startup.launch.py
```

### 4.4 start IMU kalman filter node

```bash
# launch the kalman filter node
just kflaunch
# pixi run ros2 launch zumo_launch zumo_imu_kf.launch
```

### 4.5 ROS visualization

```bash
# start rviz2 with zumo imu configuration
just rviz_imu
# pixi run ros2 run rviz2 rviz2 -d ./.rviz2/zumo_imu.rviz
```

**note:** to stop any node, press `ctrl+c` in the terminal.

---

## 5. implementation details

### 5.1 code structure

the main implementation file is located at:

```
zumo/zumo_imu_kf/src/zumo_imu_kf.cpp
```

### 5.2 functions to implement

#### roll and pitch calculation

**function:** `ZumoImuKF::TransformAccRawData`

**purpose:** transform raw accelerometer data to roll and pitch angles using the equations in section 2.3.

#### yaw calculation

**function:** `ZumoImuKF::TransformMagRawData`

**purpose:** transform raw magnetometer data to yaw angle using the equations in section 2.3 and previously calculated roll and pitch.

#### renormalization

**function:** `ZumoImuKF::Normalize()`

**purpose:** enforce orthogonality constraints on the rotation matrix using the equations in section 2.4.

#### kalman prediction

**function:** `ZumoImuKF::KalmanPredict()`

**purpose:** implement prediction equations from section 2.2 using gyroscope data to predict next rotation matrix state.

#### kalman update

**function:** `ZumoImuKF::KalmanUpdate()`

**purpose:** implement update equations from section 2.2 using observations from accelerometer and magnetometer to correct predicted state.

### 5.3 important notes

- use calibration parameters from lab 1 (accelerometer matrix and magnetometer offsets)
- copy these values into `zumo_imu_kf.launch` file or use `just update_kflaunch` to update automatically
- always verify continuous sensor data flow before starting the kalman filter
- use `just sensors` to monitor sensor topics during operation
- rviz2 visualization provides immediate feedback on orientation estimation accuracy
- the prediction step uses gyroscope data (high frequency, drifts over time)
- the update step uses accelerometer and magnetometer (low frequency, absolute reference)
- commenting out `KalmanUpdate()` demonstrates pure gyroscope integration and drift accumulation
- three independent kalman filters run simultaneously for the three rotation matrix column vectors
