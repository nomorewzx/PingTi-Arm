# URDF for Simulation

The current model is [PingTi v4 new_calib](../Simulation/PingTi_Arm_v4_new_calib/pingti_arm_v4_new_calib.urdf), using the full-length strengthened elbow-to-wrist link. Keep the URDF and its `meshes/` directory together when copying the model. Lengths are in meters, angles in radians, and masses in kilograms.

The [original v3 URDF](../Simulation/PingTi_Arm_v3/urdf/PingTi_Arm_v3.urdf) remains available. The editable CAD files are the [current Fusion assembly](../fusion/PingTi_Arm_5DOF_v4.f3d) and the [native strengthened-link source](../fusion/parts/shoulder_to_elbow_link_v2_native_source.f3d).

## Joint conventions

The v4 URDF contains five arm joints and one gripper joint. Its transforms already include the agreed nominal zeros and axis directions: the base, shoulder, elbow and wrist pitch use their CAD travel midpoints; wrist roll follows the copied SO ARM wrist reference geometry.

| URDF joint | Nominal range, degrees |
| --- | --- |
| `shoulder_pan` | -110 to 110 |
| `shoulder_lift` | -105 to 105 |
| `elbow_flex` | -100 to 100 |
| `wrist_flex` | -110 to 110 |
| `wrist_roll` | -180 to 180 |
| `gripper` | 0 to 102 |

For LeRobot, configure PingTi with `use_degrees=True`. [joint_mapping.py](../Simulation/PingTi_Arm_v4_new_calib/joint_mapping.py) converts the five arm angles from degrees to radians and maps gripper commands from 0–100 to 0–102 degrees. It accepts either joint names or observation keys such as `shoulder_pan.pos`. Do not apply the CAD offsets or signs again. The mapping does not connect to the robot or establish physical calibration.

The `tool_mount_link` frame is the output-horn reference, not a measured gripper TCP. Physical motor directions, repeatable endpoints and TCP still require measurement before Cartesian control.

## Mass model

The estimated total mass is 0.988096 kg. Printed parts use a uniform effective density of 579.725 kg/m³, inferred from the [SO101 reference model](https://github.com/TheRobotStudio/SO-ARM100/blob/608122e9ac330a753735f2e18aee73338e9ac407/Simulation/SO101/so101_new_calib.urdf). Each servo uses the same inferred mass of 54.165 g under the agreed approximation, including the STS3250 servos. These are modeling assumptions, not measured PingTi masses. Fusion's retained Steel material labels do not define the URDF mass parameters.

## Load and render

From the repository root, in a Python environment with PyBullet:

```python
import pybullet as p

p.connect(p.DIRECT)
robot = p.loadURDF(
    "Simulation/PingTi_Arm_v4_new_calib/pingti_arm_v4_new_calib.urdf",
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE,
)
p.disconnect()
```

To regenerate the README image from the actual model, install PyBullet and NumPy, then run:

```sh
python Simulation/PingTi_Arm_v4_new_calib/render_readme.py
```

The image is written to `media/pingti_arm_render.png`.
