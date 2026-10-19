"""Independent MuJoCo URDF importer, FK and mass-matrix checks. No robot I/O."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

OUT = Path(__file__).resolve().parent


def main():
    manifest = json.loads((OUT/"model_manifest.json").read_text())
    source = json.loads((OUT/"provenance/fusion_export.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(OUT/manifest["urdf"]))
    state = mujoco.MjData(model)
    base_from_cad = np.linalg.inv(np.array(manifest["cad_from_base"]))
    moving = [j for j in manifest["joints"] if j["type"] != "fixed"]
    worst_point = worst_rotation = 0.
    min_mass_eigenvalue = float("inf")
    for pose in source["poses"]:
        for joint in moving:
            q = joint["cad_to_urdf_sign"]*(pose["snapshot"]["rotations_rad"][joint["cad_joint"]]-math.radians(joint["cad_angle_at_urdf_zero_deg"]))
            if joint["name"] == "wrist_roll":
                q = math.atan2(math.sin(q), math.cos(q))
            j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
            state.qpos[model.jnt_qposadr[j_id]] = q
        mujoco.mj_forward(model, state)
        for link in manifest["links"]:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link["name"])
            predicted = np.eye(4)
            predicted[:3, :3] = state.xmat[body_id].reshape(3, 3)
            predicted[:3, 3] = state.xpos[body_id]
            predicted = predicted @ np.array(link["mesh_to_link"])
            expected = np.array(pose["snapshot"]["occurrences"][link["cad_occurrence"]]["row_major_cm"]).reshape(4, 4)
            expected[:3, 3] *= .01
            expected = base_from_cad @ expected
            worst_point = max(worst_point, float(np.linalg.norm(predicted[:3, 3]-expected[:3, 3])))
            relative = predicted[:3, :3].T @ expected[:3, :3]
            # atan2 remains well-conditioned for the very small rotations here.
            sine = np.linalg.norm(np.array([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]]))/2
            worst_rotation = max(worst_rotation, float(math.atan2(sine, (np.trace(relative)-1)/2)))
        matrix = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, matrix, state.qM)
        min_mass_eigenvalue = min(min_mass_eigenvalue, float(np.linalg.eigvalsh(matrix).min()))
        assert np.isfinite(state.qM).all()
    total_mass = float(model.body_mass.sum())
    assert abs(total_mass-manifest["mass_total_kg"]) < 1e-10
    assert model.nq == 6 and model.nv == 6
    assert worst_point < 1e-7 and worst_rotation < 1e-7
    assert min_mass_eigenvalue > 0
    # Unit conversion is checked against actual endpoint and midpoint cases.
    from joint_mapping import lerobot_to_urdf, urdf_to_lerobot, ARM_JOINTS
    mapping_cases = 0
    for opening in (0., 50., 100.):
        for degree in (-90., 0., 90.):
            values = {name+".pos": degree for name in ARM_JOINTS}
            values["gripper.pos"] = opening
            restored = urdf_to_lerobot(lerobot_to_urdf(values))
            assert max(abs(values[k]-restored[k]) for k in values) < 1e-10
            mapping_cases += 1
    result = {"status": "passed", "engine": "MuJoCo", "version": mujoco.__version__,
              "urdf_import": True, "dof": model.nq, "physical_mass_kg": total_mass,
              "fusion_poses_checked": len(source["poses"]),
              "max_component_origin_error_mm": worst_point*1000,
              "max_component_rotation_error_deg": math.degrees(worst_rotation),
              "minimum_generalized_mass_matrix_eigenvalue": min_mass_eigenvalue,
              "unit_mapping_roundtrips": mapping_cases,
              "scope": "Independent importer/FK and inertial mass matrices. No actuator tuning, collision-free planning, contact-task rollout or hardware validation."}
    (OUT/"mujoco_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
