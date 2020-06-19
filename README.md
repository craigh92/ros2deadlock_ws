# ros2deadlock_ws

### Quick Start

Build:

```bash
git clone https://github.com/craigh92/ros2deadlock_ws.git
cd ros2deadlock_ws
source ~/ros2_foxy/install/setup.bash
colcon build
```
Run:

```
source ./install/setup.sh
ros2 run my_package my_node --exe=M
```
Or
```
ros2 run my_package my_node --exe=S
```

Optional arguments
```
  --d          Print a debug message at the start and end of every spin
```
