### URDF

A PyBullet URDF is created by using [Fusion2PyBullet](https://github.com/yanshil/Fusion2PyBullet/tree/master) , see `./urdf/PingTi_Arm_pybullet/`

### Kinematics and IK

The helper script at `Simulation/PingTi_Arm_v3/kinematics.py` builds a forward and inverse kinematics model directly from the URDF so it stays aligned with the Lerobot SO100 end-effector convention. The default end-effector frame is the gripper mount with an additional offset that places the frame between the jaws, matching Lerobot’s grasp point.

Example usage:

```bash
python Simulation/PingTi_Arm_v3/kinematics.py
```

The demo prints the forward kinematics for a sample configuration and verifies the inverse kinematics solver can recover the same pose.
