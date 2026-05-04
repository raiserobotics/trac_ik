# trac_ik — ROS2 Jazzy Port

This is a minimal ROS2 Jazzy port of TRAC-IK for use with metalman2.

Source: `metalman/metalman_external/trac_ik` (ROS1 catkin baseline from TRACLabs).

## Packages

- `trac_ik_lib` — C++ kinematics library (ament_cmake). ROS logging macros replaced with
  `fprintf(stderr, ...)` / `throw std::runtime_error(...)`. The `ros::NodeHandle`-based
  constructor is removed; the URDF string must always be passed directly.
- `trac_ik_python` — Python SWIG wrapper (ament_cmake + ament_cmake_python). `rospy` dependency
  removed; callers must always supply `urdf_string` explicitly.

## Usage

```python
from trac_ik_python.trac_ik_wrap import TRAC_IK

with open("robot.urdf") as f:
    urdf = f.read()

ik = TRAC_IK("base_link", "tool0", urdf, 0.005, 1e-5, "Speed")
seed = [0.0] * ik.getNrOfJointsInChain()
result = ik.CartToJnt(seed, x, y, z, rx, ry, rz, rw, 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3)
```

## Integration

Add to `metalman2.repos`:
```yaml
  trac_ik:
    type: git
    url: https://github.com/raiserobotics/trac_ik.git
    version: main
```

Then build:
```bash
colcon build --packages-select trac_ik_lib trac_ik_python
```
