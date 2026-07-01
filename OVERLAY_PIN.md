# AS2 Platform Multirotor Simulator Overlay Pin

This overlay source package was obtained from the ROS 2 Humble Debian source
package so `rl_uav_aerostack2` can modify and build the real simulator package
instead of the launcher-only project package.

## Pin

- ROS source package: `ros-humble-as2-platform-multirotor-simulator`
- Debian source version: `1.1.3-1jammy`
- ROS package name: `as2_platform_multirotor_simulator`
- ROS package version: `1.1.3`
- Overlay path: `/home/jordi/as2_rl_ws/src/as2_platform_multirotor_simulator`
- Acquisition command: `apt-get source ros-humble-as2-platform-multirotor-simulator`
- Original extracted directory: `ros-humble-as2-platform-multirotor-simulator-1.1.3`

## Source artifacts preserved in `/home/jordi/as2_rl_ws/src`

- `ros-humble-as2-platform-multirotor-simulator_1.1.3-1jammy.dsc`
- `ros-humble-as2-platform-multirotor-simulator_1.1.3.orig.tar.gz`
- `ros-humble-as2-platform-multirotor-simulator_1.1.3-1jammy.debian.tar.xz`

## Checksums from `.dsc`

- `ros-humble-as2-platform-multirotor-simulator_1.1.3.orig.tar.gz`
  - SHA256: `1ad11d9f34a569076c8e3c09d8385559acc9ecb63de6aee77819e6cce92f96d7`
- `ros-humble-as2-platform-multirotor-simulator_1.1.3-1jammy.debian.tar.xz`
  - SHA256: `df8155d5fdc2f3c0cb07633fe78c9e5c5629678cb0a6309776df69f632fc653e`

## Overlay discovery command

Use the project conda environment and source ROS 2 Humble before checking the
overlay package list:

```bash
conda run -n rl_uav bash -lc 'source /opt/ros/humble/setup.bash && colcon list --base-paths /home/jordi/as2_rl_ws/src --names-only'
```

Expected output includes:

```text
as2_platform_multirotor_simulator
```
