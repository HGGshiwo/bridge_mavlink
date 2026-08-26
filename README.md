# bridge_mavlink

## NOTE

依赖：

```bash
sudo apt-get install ros-noetic-mavros
sudo apt-get install ros-noetic-mavros-extras
```

以及以下第三方库（在编译时会自动配置/获取）：
- **Eigen3** (标准矩阵/代数库)
- **nlohmann_json** (自动通过 CMake `FetchContent` 拉取)

对于真机：

```
roslaunch bridge_mavlink bridge_mavlink.launch fcu_url:=/dev/ttyS5:115200
```

ROS Noetic C++ node that bridges telemetry data from MAVROS, formatting it into a JSON-serialized string and publishing it on the `/dank/status` topic.

## Telemetry Field Mapping

The node outputs a `std_msgs/String` containing the following fields:

* **`pos_enu`**: `[x, y, z]` coordinates in the ENU (East-North-Up) frame.
* **`roll`**, **`pitch`**, **`yaw`**: Euler attitude angles (in radians by default, or degrees if configured).
* **`gps`**: `[lon, lat, alt]` representing longitude, latitude, and altitude.
* **`gps_fix_type`**: MAVLink GPS fix type integer (e.g. `3` for 3D Fix, `6` for RTK Fixed).

### JSON Output Example

```json
{
  "pos_enu": [0.05, 1.23, 0.54],
  "roll": 0.015,
  "pitch": -0.024,
  "yaw": 1.571,
  "gps": [116.397, 39.908, 52.4],
  "gps_fix_type": 3
}
```

---

## Installation & Compilation

Ensure your workspace contains `mavros_msgs` (or it is installed on the system). Then compile the workspace:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps bridge_mavlink
# Or using catkin build:
# catkin build bridge_mavlink

source devel/setup.bash
```

---

## Running the Bridge (Including MAVROS)

The provided launch file launches **both MAVROS and the bridge node** together.

```bash
roslaunch bridge_mavlink bridge_mavlink.launch
```

### Configurable Launch Arguments

You can configure the MAVROS connection url and parameters directly through launch arguments:

```bash
roslaunch bridge_mavlink bridge_mavlink.launch fcu_url:=udp://:14540@127.0.0.1:14557 publish_rate:=10.0
```

#### MAVROS Settings
* `fcu_url` (default: `udp://:14540@127.0.0.1:14557`) — Connection URL to flight controller unit.
* `gcs_url` (default: `""`) — Connection URL to ground control station.
* `tgt_system` (default: `1`) — System ID of target device.
* `tgt_component` (default: `1`) — Component ID of target device.
* `fcu_protocol` (default: `v2.0`) — MAVLink protocol version.

#### Bridge Settings
* `publish_rate` (default: `20.0`) — Output update rate in Hz.
* `use_degrees` (default: `false`) — Convert Euler angles to degrees if set to `true`.
* `odom_topic` (default: `/mavros/local_position/odom`) — The ROS topic to subscribe to for local positioning ENU data (must be of type `nav_msgs/Odometry`).

### Switching Local Odometry Topic
You can configure the source of local positioning ENU data using the `odom_topic` argument:

- **Read from flight controller (default):**
  ```bash
  roslaunch bridge_mavlink bridge_mavlink.launch odom_topic:=/mavros/local_position/odom
  ```
- **Read from external local odometry (e.g. `/loc_base`):**
  ```bash
  roslaunch bridge_mavlink bridge_mavlink.launch odom_topic:=/loc_base
  ```
- **Any other odometry source:**
  Pass the name of any topic publishing `nav_msgs/Odometry` messages (e.g., `odom_topic:=/my_custom/odometry`).

---

## Features

### GPS Loss Protection (Forged GPS)
When the GPS fix is lost or unavailable (`gps_fix_type < 3`), the node automatically estimates and publishes a forged GPS coordinate `[lon, lat, alt]`. This estimation is based on:
1. The last known reliable GPS-ENU alignment datum (synchronized using the `DatumSynchronizer` while `gps_fix_type >= 3`).
2. The current local ENU position offset relative to that datum.

This ensures the `/dank/status` JSON output stream contains continuous, uninterrupted geographic coordinate estimates.

---

## Verification

To verify that the bridge is running correctly and producing the JSON payloads, run:

```bash
rostopic echo /dank/status
```
