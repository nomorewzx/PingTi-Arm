"""Kinematics utilities for the PingTi arm.

The implementation follows the Lerobot SO100 end-effector convention: the
end-effector pose is reported at the gripper mounting frame, with an optional
offset that points to the same fingertip location used by the SO100 bridge.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class JointSpec:
    """Minimal joint description parsed from the URDF."""

    name: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: Optional[np.ndarray]
    joint_type: str
    limit: Optional[Tuple[float, float]] = None


# A fixed offset from the gripper mount to the nominal end-effector point used
# by the Lerobot SO100 integration. The translation is taken from the
# ``gripper_moving`` joint origin in the URDF so that the frame sits between the
# jaws even when the gripper angle is zero.
LEROBOT_EE_OFFSET = np.array([0.019807, 0.003967, 0.0352])


# Utility functions -----------------------------------------------------------------

def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """Convert roll, pitch, yaw to a rotation matrix."""

    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula for a normalized axis."""

    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def transform_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Compose a 4x4 homogeneous transform from rotation and translation."""

    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


# URDF parsing ----------------------------------------------------------------------

def parse_urdf_joints(urdf_path: Path) -> Dict[str, JointSpec]:
    """Parse the joints from the URDF into a dictionary."""

    root = ET.parse(urdf_path).getroot()
    joints: Dict[str, JointSpec] = {}

    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        joint_type = joint_el.get("type", "fixed")
        parent = joint_el.find("parent").get("link")  # type: ignore[arg-type]
        child = joint_el.find("child").get("link")  # type: ignore[arg-type]

        origin_el = joint_el.find("origin")
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        if origin_el is not None:
            xyz = np.array([float(v) for v in origin_el.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in origin_el.get("rpy", "0 0 0").split()])

        axis_el = joint_el.find("axis")
        axis = None
        if axis_el is not None:
            axis = np.array([float(v) for v in axis_el.get("xyz", "0 0 1").split()])

        limit_el = joint_el.find("limit")
        joint_limit = None
        if limit_el is not None:
            lower = float(limit_el.get("lower", "0"))
            upper = float(limit_el.get("upper", "0"))
            joint_limit = (lower, upper)

        joints[name] = JointSpec(
            name=name,
            parent=parent,
            child=child,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=axis,
            joint_type=joint_type,
            limit=joint_limit,
        )

    return joints


def _build_child_map(joints: Dict[str, JointSpec]) -> Dict[str, List[str]]:
    child_map: Dict[str, List[str]] = {}
    for name, spec in joints.items():
        child_map.setdefault(spec.parent, []).append(name)
    return child_map


def _find_chain(
    joints: Dict[str, JointSpec],
    base_link: str,
    target_link: str,
) -> List[str]:
    """Find the ordered joint names connecting ``base_link`` to ``target_link``."""

    child_map = _build_child_map(joints)

    # Depth-first search over joints
    stack: List[Tuple[str, List[str]]] = [(base_link, [])]
    visited: set[str] = set()

    while stack:
        link, path = stack.pop()
        if link == target_link:
            return path
        if link in visited:
            continue
        visited.add(link)

        for joint_name in child_map.get(link, []):
            joint = joints[joint_name]
            stack.append((joint.child, path + [joint_name]))

    raise ValueError(f"No kinematic chain found from {base_link} to {target_link}")


# Kinematics -----------------------------------------------------------------------


class RobotKinematics:
    """Forward and inverse kinematics built directly from the URDF."""

    def __init__(
        self,
        urdf_path: Path = Path(__file__).parent / "urdf" / "PingTi_Arm_v3.urdf",
        base_link: str = "base_link",
        end_link: str = "gripper_mount_1",
        active_joints: Sequence[str] = (
            "base_yaw",
            "shoulder_pitch",
            "elbow_pitch",
            "wrist_pitch",
            "wrist_roll",
        ),
    ) -> None:
        self.urdf_path = urdf_path
        self.joints = parse_urdf_joints(self.urdf_path)
        self.base_link = base_link
        self.end_link = end_link
        self.active_joints = list(active_joints)

        self.chain = _find_chain(self.joints, base_link, end_link)
        self.joint_limits = {
            name: self.joints[name].limit for name in self.active_joints if self.joints[name].limit
        }

    def origin_transform(self, joint: JointSpec) -> np.ndarray:
        return transform_matrix(rpy_to_matrix(joint.origin_rpy), joint.origin_xyz)

    def joint_transform(self, joint: JointSpec, angle: float) -> np.ndarray:
        if joint.joint_type == "fixed" or joint.axis is None:
            return self.origin_transform(joint)
        return self.origin_transform(joint) @ transform_matrix(
            axis_angle_rotation(joint.axis, angle), np.zeros(3)
        )

    def forward(
        self,
        joint_angles: Dict[str, float],
        *,
        link: Optional[str] = None,
        lerobot_tip: bool = True,
    ) -> np.ndarray:
        """Compute the homogeneous transform of the requested link.

        Args:
            joint_angles: Mapping from joint name to angle (radians).
            link: Target link name. Defaults to the configured ``end_link``.
            lerobot_tip: When ``True``, append the SO100-compatible fingertip offset
                so the resulting transform lines up with Lerobot's end-effector
                convention. Set to ``False`` to return the raw link pose.
        """

        target_link = link or self.end_link
        chain = self.chain if target_link == self.end_link else _find_chain(self.joints, self.base_link, target_link)

        T = np.eye(4)
        for joint_name in chain:
            joint = self.joints[joint_name]
            angle = joint_angles.get(joint_name, 0.0)
            T = T @ self.joint_transform(joint, angle)

        if lerobot_tip and target_link == self.end_link:
            T = T @ transform_matrix(np.eye(3), LEROBOT_EE_OFFSET)
        return T

    def numerical_jacobian(
        self, joint_angles: Dict[str, float], delta: float = 1e-4
    ) -> np.ndarray:
        """Finite-difference position jacobian for the active joints."""

        base_pose = self.forward(joint_angles)
        base_pos = base_pose[:3, 3]
        J = np.zeros((3, len(self.active_joints)))

        for idx, name in enumerate(self.active_joints):
            perturbed = dict(joint_angles)
            perturbed[name] = perturbed.get(name, 0.0) + delta
            pose_delta = self.forward(perturbed)
            J[:, idx] = (pose_delta[:3, 3] - base_pos) / delta

        return J

    def inverse(
        self,
        target: np.ndarray,
        *,
        initial_guess: Optional[Dict[str, float]] = None,
        max_iters: int = 200,
        position_tolerance: float = 1e-4,
        damping: float = 1e-3,
    ) -> Dict[str, float]:
        """Solve inverse kinematics using damped least squares.

        Orientation matching is intentionally omitted because the PingTi arm's
        primary Lerobot workflows only require positional control of the SO100
        style end-effector. The solution respects the URDF joint limits when
        available.
        """

        if initial_guess is None:
            initial_guess = {name: 0.0 for name in self.active_joints}
        current = dict(initial_guess)

        for _ in range(max_iters):
            pose = self.forward(current)
            position_error = target[:3, 3] - pose[:3, 3]
            if np.linalg.norm(position_error) < position_tolerance:
                break

            J = self.numerical_jacobian(current)
            JT = J.T
            # Damped least squares update
            dq = JT @ np.linalg.inv(J @ JT + damping * np.eye(3)) @ position_error

            for angle, name in zip(dq, self.active_joints):
                current[name] = current.get(name, 0.0) + float(angle)
                lower_upper = self.joint_limits.get(name)
                if lower_upper is not None:
                    lower, upper = lower_upper
                    current[name] = float(np.clip(current[name], lower, upper))

        return current


def demo() -> None:
    robot = RobotKinematics()
    seed = {
        "base_yaw": 0.0,
        "shoulder_pitch": 0.2,
        "elbow_pitch": 0.4,
        "wrist_pitch": 0.0,
        "wrist_roll": 0.0,
    }
    pose = robot.forward(seed)
    print("Forward kinematics (with Lerobot offset):")
    print(np.array_str(pose, precision=4))

    # Solve IK back to the same position from a different start
    target = pose.copy()
    guess = {name: 0.0 for name in robot.active_joints}
    solution = robot.inverse(target, initial_guess=guess)
    solved_pose = robot.forward(solution)
    print("\nRecovered joint angles:", solution)
    print("Pose error (m):", np.linalg.norm(target[:3, 3] - solved_pose[:3, 3]))


if __name__ == "__main__":
    demo()
