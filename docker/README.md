# Docker workflow

This directory provides a headless Docker development environment for the ROS2 Adaptive Assembly project.

The container is intended for:

- reproducible ROS2 Jazzy + MoveIt2 builds;
- plan-only adaptive assembly demos;
- validation scripts;
- benchmark recording and comparison tools.

The container is not intended for:

- RViz2 GUI rendering;
- Gazebo GUI rendering;
- NVIDIA GPU acceleration;
- real robot hardware access;
- camera/USB device passthrough;
- contact-rich insertion simulation.
---
## Structure
```
/workspaces/ros2_adaptive_assembly_ws/
├── build/                         # colcon build output，no Git
├── install/                       # colcon install output，no Git
├── log/                           # colcon log output，no Git
└── src/
    ├── ros2_adaptive_assembly/    #  GitHub repo
    │   ├── README.md
    │   ├── docker/
    │   ├── docs/
    │   ├── scripts/
    │   └── src/                   #  ROS2 packages
    └── third_party/
        └── moveit_resources/    
```

---
## Build and start

From the repository root:

```bash
docker compose -f docker/compose.yaml up -d --build dev
```
Enter the container:
```bash
docker compose -f docker/compose.yaml exec dev bash
```
## First build inside the container
```bash
cd /workspaces/ros2_adaptive_assembly_ws

source /opt/ros/jazzy/setup.bash 
mkdir -p src/third_party
cd src/third_party/
git clone https://github.com/moveit/moveit_resources.git

apt-get update
rosdep update

rosdep install \
  --from-paths src/ros2_adaptive_assembly/src src/third_party/moveit_resources \
  --ignore-src \
  --skip-keys "moveit_resources_panda_moveit_config" \
  -r -y

colcon build --symlink-install \
  --base-paths src/ros2_adaptive_assembly/src src/third_party/moveit_resources

source install/setup.bash
```
## Run the recommended smoke test
```bash
bash scripts/docker_smoke_test.sh
```
## Manual two-terminal demo
Terminal A:
```
docker compose -f docker/compose.yaml exec dev bash
source install/setup.bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_reachable.launch.py
```
Terminal B:
```
docker compose -f docker/compose.yaml exec dev bash
source install/setup.bash
python3 scripts/check_assembly_sequence_success_path.py
```
## Stop the container
```
docker compose -f docker/compose.yaml down
```
Remove build/install/log volumes if you want a completely clean rebuild:
```
docker compose -f docker/compose.yaml down -v
```
