

---

# arm24_servo

ROS Noetic package for Cartesian velocity control of 6-DOF robotic arm using MoveIt Servo.

## Features

- Real-time Cartesian teleop (keyboard control)
- MoveIt Servo integration with speed_units mode
- Serial bridge for STM32 microcontroller communication
- Dictionary-based joint mapping (order-independent)
- Smooth acceleration/deceleration

## Requirements

- Ubuntu 20.04 / ROS Noetic
- MoveIt & MoveIt Servo
- Robot URDF/SRDF definition
- Python 3

## Installation

**1. Clone the package:**
```bash
cd ~/catkin_ws/src
git clone git@github.com:atahanyp/arm24_servo.git
```

**2. Install MoveIt Servo:**

Via APT (if available):
```bash
sudo apt update
sudo apt install -y ros-noetic-moveit-servo ros-noetic-moveit
```

Or build from source:
```bash
cd ~/catkin_ws/src
git clone -b master https://github.com/ros-planning/moveit.git
```

**3. Install dependencies and build:**
```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build  # or catkin_make
source devel/setup.bash
```

## Usage

**1. Launch servo server:**
```bash
roslaunch arm24_servo servo.launch
```

**2. Enable velocity mode:**
```bash
rosservice call /hardware_interface/velocity_mode "data: true"
```

**3. Run teleop:**
```bash
rosrun arm24_servo twist_test.py
```

**Controls:**
- `w/s`: Forward/Back (X)
- `a/d`: Left/Right (Y)
- `r/f`: Up/Down (Z)
- `u/o`: Roll, `i/k`: Pitch, `j/l`: Yaw
- `=/-`: Adjust step size
- `space`: Stop, `q`: Quit

## Configuration

**Max velocities** in `scripts/servo_to_serial_bridge.py`:
```python
self.max_vel_rad = {
    "joint1": 1.5,  # rad/s
    "joint2": 1.5,
    # ... adjust for your robot
}
```

**Servo parameters** in `config/naim24_servo.yaml`:
```yaml
move_group_name: manipulator
planning_frame: arm_base
ee_frame_name: end_effector
command_in_type: "speed_units"
```

## Data Flow

```
twist_test.py → MoveIt Servo → servo_to_serial_bridge → rk23_serial → STM32
```

## Package Structure

```
arm24_servo/
├── config/naim24_servo.yaml      # Servo parameters
├── launch/servo.launch           # Main launch file
├── scripts/
│   ├── twist_test.py             # Cartesian teleop
│   ├── servo_to_serial_bridge.py # Encoder (rad/s → STM format)
│   └── wrist_singguard.py        # Wrist singularity guard (disabled)
└── README.md
```

## Notes

- For first tests, reduce `step` value in `twist_test.py`
- STM32 watchdog recommended for safety
- Update `joint_limits.yaml` for your robot
- Joint order is automatically matched via dictionary mapping

## License

MIT

## Contact

ITU Rover Team


