"""Independent PyBullet loading/FK checks and CPU renders. No robot I/O."""

import json
import math
from pathlib import Path
import struct
import zlib

import numpy as np
import pybullet as p

OUT = Path(__file__).resolve().parent


def png(path, rgba):
    array = np.asarray(rgba, dtype=np.uint8)
    height, width = array.shape[:2]
    def chunk(kind, data):
        return struct.pack(">I", len(data))+kind+data+struct.pack(">I", zlib.crc32(kind+data)&0xffffffff)
    raw = b"".join(b"\x00"+row.tobytes() for row in array)
    path.write_bytes(b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))+
                     chunk(b"IDAT", zlib.compress(raw))+chunk(b"IEND", b""))


def main():
    manifest = json.loads((OUT/"model_manifest.json").read_text())
    source = json.loads((OUT/"provenance/fusion_export.json").read_text())
    connection = p.connect(p.DIRECT)
    robot = p.loadURDF(str(OUT/manifest["urdf"]), useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
    joints = {p.getJointInfo(robot, i)[1].decode(): i for i in range(p.getNumJoints(robot))}
    link_ids = {"base_link": -1, **{p.getJointInfo(robot, i)[12].decode(): i for i in range(p.getNumJoints(robot))}}
    moving = [j for j in manifest["joints"] if j["type"] != "fixed"]
    base_from_cad = np.linalg.inv(np.array(manifest["cad_from_base"]))
    max_error = 0.
    for pose in source["poses"]:
        for joint in moving:
            value = joint["cad_to_urdf_sign"]*(pose["snapshot"]["rotations_rad"][joint["cad_joint"]]-math.radians(joint["cad_angle_at_urdf_zero_deg"]))
            if joint["name"] == "wrist_roll":
                value = math.atan2(math.sin(value), math.cos(value))
            p.resetJointState(robot, joints[joint["name"]], value)
        for link in manifest["links"]:
            link_id = link_ids[link["name"]]
            predicted = np.eye(4)
            if link_id >= 0:
                state = p.getLinkState(robot, link_id, computeForwardKinematics=True)
                predicted[:3, 3] = state[4]
                predicted[:3, :3] = np.array(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
            predicted = predicted @ np.array(link["mesh_to_link"])
            expected = np.array(pose["snapshot"]["occurrences"][link["cad_occurrence"]]["row_major_cm"]).reshape(4, 4)
            expected[:3, 3] *= .01
            expected = base_from_cad @ expected
            points = np.array([[0.,0.,0.,1.],[.1,0.,0.,1.],[0.,.1,0.,1.],[0.,0.,.1,1.]])
            max_error = max(max_error, float(np.linalg.norm((points@predicted.T-points@expected.T)[:,:3], axis=1).max()))
    assert max_error < 2e-6
    # Fixed-base PyBullet represents the grounded base mass as zero. Compare
    # every movable/fixed descendant's imported mass individually instead.
    for link in manifest["links"]:
        if link_ids[link["name"]] >= 0:
            assert abs(p.getDynamicsInfo(robot, link_ids[link["name"]])[0]-link["mass_kg"]) < 1e-12
    assert p.getDynamicsInfo(robot, link_ids["tool_mount_link"])[0] == 0.
    renders = []
    poses = [("zero_pose", {}, [0.95,-1.0,.80]),
             ("zero_pose_side", {}, [.22,-1.1,.34]),
             ("gripper_open", {"gripper": 102.}, [.95,-1.0,.80])]
    floor_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[.75,.60,.003], rgbaColor=[.91,.93,.95,1.])
    p.createMultiBody(baseMass=0., baseVisualShapeIndex=floor_visual, basePosition=[.25,0.,-.006])
    for label, degrees, eye in poses:
        for joint in moving:
            p.resetJointState(robot, joints[joint["name"]], math.radians(degrees.get(joint["name"], 0.)))
        view = p.computeViewMatrix(eye, [.20,0.,.24], [0.,0.,1.])
        projection = p.computeProjectionMatrixFOV(42., 4/3, .01, 4.)
        image = p.getCameraImage(1280,960, viewMatrix=view, projectionMatrix=projection,
                                 renderer=p.ER_TINY_RENDERER, lightDirection=[-1,-2,4], shadow=1)
        filename = label+".png"
        png(OUT/filename, image[2])
        renders.append(filename)
    result = {"status": "passed", "engine": "PyBullet", "api_version": p.getAPIVersion(),
              "fusion_poses_checked": len(source["poses"]), "max_rigid_reference_point_error_mm": max_error*1000,
              "imported_descendant_masses_match": True, "tool_mount_mass_kg": 0.,
              "renders": renders, "renderer": "CPU TinyRenderer",
              "scope": "Second independent URDF importer/FK and visual inspection; hardware and contact dynamics are outside this check."}
    (OUT/"pybullet_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    p.disconnect(connection)


if __name__ == "__main__":
    main()
