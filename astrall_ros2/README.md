# Astrall ROS2 Layer

This directory is the ROS2 adaptation layer. It should not absorb the core C++ runtime or the pybind11 module.

Production navigation is fixed to this responsibility chain:

```text
LiDAR ROS2 driver
  -> PointCloud2
  -> FAST-LIO/localization
  -> odom + TF
  -> Nav2
  -> /cmd_vel
  -> astrall_base_driver
  -> astrall_core Backend
  -> RealBackend
  -> Astrall SDK
```

Core `Planner`, `Controller`, `Navigator`, and `StateMachine` remain useful for demos, simulation, and minimal non-ROS runtime paths. They are not Nav2 replacements.

## Conceptual Nodes

```text
RadarNode
  astrall_ros2/radar_node.
  External LiDAR driver, vendor SDK, or UDP parser.
  Publishes /front/points_raw, /rear/points_raw, /front/imu, /rear/imu.
  Does not use AstrallSubscriptionData for point clouds.

CameraNode
  astrall_ros2/camera_node.
  ROS2 camera adapter when production camera transport is added.

ControllerNode
  astrall_ros2/controller_node, packaged as astrall_base_driver.
  Subscribes /cmd_vel, clamps/maps Twist, sends commands through astrall_core Backend,
  and publishes Backend IMU, wheel speeds, robot status, and diagnostics.
  It is a base driver, not a Nav2 controller, and it should not construct Runtime.

OdomNode
  astrall_ros2/odom_node.
  Localization output owner, typically FAST-LIO or another estimator.
  Consumes PointCloud2 + IMU and publishes odom.

TfNode
  astrall_ros2/tf_node, packaged as astrall_description for now.
  Static sensor transforms plus the simplified RViz robot description.

SimNode
  astrall_ros2/sim_node, packaged as astrall_sim.
  Lightweight RViz simulation: subscribes /cmd_vel, advances SimBackend,
  publishes /odom, odom->base_link TF, /map, simulated front/rear PointCloud2,
  simulated IMU, wheel speeds, status, and diagnostics. It does not use the
  Astrall SDK, and its PointCloud2 topics are simulation-only.
```

## LiDAR Network

```text
Front LiDAR 10.18.0.120
  MSOP 6699   -> point cloud UDP
  DIFOP 7788  -> device/config UDP
  IMU  6688   -> LiDAR internal IMU

Rear LiDAR 10.18.0.121
  MSOP 6969   -> point cloud UDP
  DIFOP 7878  -> device/config UDP
  IMU  6868   -> LiDAR internal IMU
```

Expected ROS2 topics:

- `/front/points_raw`: `sensor_msgs/msg/PointCloud2`
- `/rear/points_raw`: `sensor_msgs/msg/PointCloud2`
- `/front/imu`: `sensor_msgs/msg/Imu`
- `/rear/imu`: `sensor_msgs/msg/Imu`

FAST-LIO consumes PointCloud2 plus IMU and outputs odom/TF. Nav2 consumes PointCloud2 through obstacle or voxel layers and outputs `/cmd_vel`. The Astrall SDK bridge receives `/cmd_vel`, maps it to `Twist2D{vx, vy, w}`, and forwards it to the SDK adapter.

## Implementation Status

| Area | Status |
| --- | --- |
| Base driver | Implemented as `astrall_base_driver`: `/cmd_vel` input, Backend velocity output, IMU/wheel/status/diagnostics publishers, monitor mode, timeout stop, and no-control-authority guard. |
| RViz simulation | Implemented as `astrall_sim`: `/cmd_vel` input, `SimBackend` odom/TF, simplified robot dog URDF, simulated `/map`, simulated `/front/points_raw` and `/rear/points_raw`, and RViz launch. |
| LiDAR boundary | Documented network and topics. Production PointCloud2 must come from an external ROS2 driver, vendor SDK, or UDP parser. |
| Localization/navigation | External: FAST-LIO or equivalent estimator plus Nav2. This layer does not publish odom localization TF. |
| Future work | Production camera transport, deployment launch composition, and ROS2 integration tests with fake Backend/SDK. |

## RViz Simulation

Build the description and simulation packages in a ROS2 workspace:

```bash
colcon build --base-paths astrall_ros2/tf_node astrall_ros2/sim_node --packages-select astrall_description astrall_sim
```

Launch the lightweight simulation:

```bash
ros2 launch astrall_sim astrall_sim.launch.py
```

Publish velocity commands to move the model:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"
```

The simulation publishes `/map`, `/front/points_raw`, and `/rear/points_raw`
for RViz inspection. These are generated from a small built-in test world and
are not a replacement for the production LiDAR driver.
