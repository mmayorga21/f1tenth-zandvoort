# F1TENTH — Zandvoort, Part 1

## 1. Approach

This project uses Follow the Gap, a reactive navigation algorithm that selects the vehicle's steering and speed from LiDAR distance measurements.

The controller processes the readings, adds a safety margin around obstacles, and identifies contiguous areas of free space. It selects a target within the chosen gap and steers towards it. Speed is adjusted to the required turn and the available clearance ahead.

The default profile allows up to **12 m/s**, with speed reduced according to curvature and braking distance.

The vehicle responds to the observed surroundings without following a predefined driving trajectory. A separate lap tracker uses odometry to count complete circuits and measure lap times. On the final lap, the simulator limits speed near the finish gate so the vehicle can stop just beyond it.

## 2. Code structure

The main project files are organized as follows:

```text
F1Tenth-Repository/
├── README.md
├── scripts/                         # Validation scripts
└── src/
    ├── f1tenth_controller/
    │   ├── config/
    │   │   ├── follow_the_gap.yaml   # Controller parameters
    │   │   ├── conservative.yaml     # Conservative speed profile
    │   │   └── lap_tracker.yaml      # Lap counter configuration
    │   ├── f1tenth_controller/
    │   │   ├── gap.py                # Follow the Gap algorithm
    │   │   ├── gap_node.py           # Controller ROS interface
    │   │   ├── control_gate.py       # Controller activation and deactivation
    │   │   ├── laps.py               # Lap counting and timing
    │   │   ├── lap_node.py           # Lap tracker ROS node
    │   │   ├── lap_panel.py          # Terminal lap and timing display
    │   │   ├── lap_display_node.py   # Additional read-only terminal viewer
    │   │   └── session_guard.py      # Attempt validation checks
    │   ├── launch/                   # Launch files
    │   └── test/                     # Controller and lap counter tests
    └── f1tenth_gym_ros/
        ├── config/
        │   └── sim.yaml             # Simulator configuration
        ├── maps/
        │   ├── Zandvoort_map.png     # Track image
        │   └── Zandvoort_map.yaml    # Map resolution and origin
        ├── f1tenth_gym_ros/
        │   ├── gym_bridge.py         # Simulator integration with ROS
        │   └── simulation_session.py # Attempt states and termination
        └── launch/
            ├── gym_bridge_launch.py  # Simulator and RViz launch file
            └── gym_bridge.rviz       # Visualization settings
```

The repository also retains packages from the original simulator. This project primarily uses `f1tenth_controller` and `f1tenth_gym_ros`.

## 3. Execution instructions

These instructions assume **Ubuntu 22.04, ROS 2 Humble, and F1TENTH Gym are installed**, together with `rosdep` and `colcon`.

### 3.1. Clone and prepare the project

Open a terminal and clone the repository:

```bash
cd ~
git clone https://github.com/mmayorga21/f1tenth-zandvoort.git F1Tenth-Repository
cd ~/F1Tenth-Repository
```

Before building, open the simulator configuration:

```bash
nano src/f1tenth_gym_ros/config/sim.yaml
```

Set `map_path` to the project's location on your computer. Replace `YOUR_USERNAME` with your Ubuntu username:

```yaml
map_path: '/home/YOUR_USERNAME/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Zandvoort_map'
```

Write this path **without the `.png` or `.yaml` extension**.

Install the ROS dependencies and build the workspace:

```bash
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

colcon build --symlink-install
source install/setup.bash
```

### 3.2. Start the simulator

In **terminal 1**, load the environment:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
```

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Wait for RViz to display Zandvoort and the vehicle. Keep this terminal open.

https://github.com/user-attachments/assets/1d38aa8d-d977-45f5-8e1f-aa6f435003ab

### 3.3. Start the controller and lap tracker

Open **terminal 2** for the controller:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
```

```bash
ros2 launch f1tenth_controller follow_the_gap.launch.py
```

The controller starts disabled, so the vehicle will remain stationary until explicitly enabled.

https://github.com/user-attachments/assets/fda90ad6-1573-44f2-9abe-b83f9fd9939c

Open **terminal 3** for the lap counter and timer:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run f1tenth_controller lap_tracker --ros-args \
  --params-file install/f1tenth_controller/share/f1tenth_controller/config/lap_tracker.yaml
```

This terminal displays completed laps, individual lap times, the best lap, and the accumulated time. Keep the controller and lap tracker running throughout the attempt.

The recording below predates the English translation of the lap panel. Current terminal output is in English.

https://github.com/user-attachments/assets/42a19ed6-84ac-4e24-83c3-7dfc1afe9e51

### 3.4. Run the ten-lap attempt

Open **terminal 4**:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Reset the simulator to return the vehicle to its starting pose and prepare a new attempt:

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
```

After the reset completes, enable the controller:

```bash
ros2 service call /controller/enable std_srvs/srv/SetBool "{data: true}"
```

The vehicle will start driving autonomously. After ten valid laps, it will brake and the attempt will finish automatically. The panel will retain the lap times and display the completion message.

To run another attempt, repeat the reset and enable commands in that order.

The following video shows the start commands and the first seconds of driving:

https://github.com/user-attachments/assets/1b382e8a-57e8-459e-8228-c0e61c74e8b7

### 3.5. Stop execution

To stop the vehicle before completing ten laps:

```bash
ros2 service call /controller/enable std_srvs/srv/SetBool "{data: false}"
```

To close the programs, press `Ctrl+C` in the controller, lap tracker, and simulator terminals.
