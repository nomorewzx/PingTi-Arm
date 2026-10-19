"""LeRobot action/observation units to this URDF. No robot I/O.

The five arm inputs must come from PingTi configured with use_degrees=True.
The URDF already contains the nominal zero and direction transforms; do not
apply the CAD offsets or signs again to LeRobot joint values.
"""

import math

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINT_NAMES = ARM_JOINTS + ("gripper",)
GRIPPER_TRAVEL_RAD = math.radians(102.)


def lerobot_to_urdf(values):
    """Return named URDF radians from named LeRobot degrees and gripper 0..100.

    Accepts either `shoulder_pan` or `shoulder_pan.pos` keys. Secondary motor
    readings are deliberately unused: paired servos are one mechanical joint.
    The gripper mapping assumes the CAD endpoint convention documented in the
    model manifest; it is not an empirical finger-width calibration.
    """
    def value(name):
        v = float(values[name] if name in values else values[name + ".pos"])
        if not math.isfinite(v):
            raise ValueError("Non-finite position for " + name)
        return v
    result = {name: math.radians(value(name)) for name in ARM_JOINTS}
    opening = value("gripper")
    if not 0. <= opening <= 100.:
        raise ValueError("Gripper opening must be in [0, 100]")
    result["gripper"] = opening / 100. * GRIPPER_TRAVEL_RAD
    return result


def urdf_to_lerobot(values):
    """Convert named URDF radians to LeRobot degree/opening values."""
    result = {name + ".pos": math.degrees(float(values[name])) for name in ARM_JOINTS}
    result["gripper.pos"] = float(values["gripper"]) / GRIPPER_TRAVEL_RAD * 100.
    if not all(math.isfinite(value) for value in result.values()):
        raise ValueError("Non-finite position")
    return result
