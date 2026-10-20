"""Audit imported PingTi CAD without saving or editing geometry.

Exports metadata, then temporarily drives joints and restores the saved pose.
"""

import datetime
import hashlib
import importlib.util
import json
import math
import random
from pathlib import Path
import traceback

import adsk.core
import adsk.fusion


OUT = Path(__file__).resolve().parent.parent
SOURCE = OUT.parents[2] / "fusion/archive/PingTi_Arm_5DOF_v4_original.f3d"


def baseline_source():
    helper = OUT.parents[2] / "Simulation/PingTi_Arm_v4_new_calib/source_artifacts.py"
    spec = importlib.util.spec_from_file_location("pingti_git_sources", helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ensure_baseline_f3d()


def read(fn):
    try:
        return fn()
    except Exception as exc:
        return {"error": str(exc)}


def vec(obj, scale=1.0):
    return [v * scale for v in obj.asArray()] if obj else None


def matrix(obj):
    if obj is None:
        return None
    return {
        "row_major_cm": list(obj.asArray()),
        "translation_m": vec(obj.translation, 0.01),
    }


def geometry(obj):
    if obj is None:
        return None
    if obj.objectType == adsk.fusion.JointOrigin.classType():
        obj = obj.geometry
    return {
        "object_type": obj.objectType,
        "origin_m": read(lambda: vec(obj.origin, 0.01)),
        "primary_axis": read(lambda: vec(obj.primaryAxisVector)),
        "secondary_axis": read(lambda: vec(obj.secondaryAxisVector)),
        "third_axis": read(lambda: vec(obj.thirdAxisVector)),
    }


def props(obj):
    p = obj.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    return {
        "mass_kg": p.mass,
        "volume_cm3": p.volume,
        "density_kg_per_cm3": p.density,
        "center_of_mass_m": vec(p.centerOfMass, 0.01),
        "inertia_world_kg_cm2_success_xx_yy_zz_xy_yz_xz": list(p.getXYZMomentsOfInertia()),
    }


def occurrence(o):
    bodies = []
    for b in o.bRepBodies:
        bodies.append({
            "name": b.name,
            "material_name": read(lambda: b.material.name),
            "physical_properties": read(lambda: props(b)),
            "bbox_m": read(lambda: [vec(b.boundingBox.minPoint, 0.01), vec(b.boundingBox.maxPoint, 0.01)]),
        })
    return {
        "name": o.name,
        "full_path": o.fullPathName,
        "component_name": o.component.name,
        "grounded": read(lambda: o.isGrounded),
        "transform2": read(lambda: matrix(o.transform2)),
        "legacy_transform": read(lambda: matrix(o.transform)),
        "physical_properties": read(lambda: props(o)),
        "bbox_m": read(lambda: [vec(o.boundingBox.minPoint, 0.01), vec(o.boundingBox.maxPoint, 0.01)]),
        "bodies": bodies,
    }


def limits(l):
    return {k: read(lambda k=k: getattr(l, k)) for k in (
        "isMinimumValueEnabled", "isMaximumValueEnabled", "isRestValueEnabled",
        "minimumValue", "maximumValue", "restValue",
    )}


def joint(j, kind):
    m = j.jointMotion
    result = {
        "name": j.name,
        "kind": kind,
        "motion_type": m.jointType,
        "motion_object_type": m.objectType,
        "occurrence_one": read(lambda: j.occurrenceOne.fullPathName),
        "occurrence_two": read(lambda: j.occurrenceTwo.fullPathName),
        "suppressed": read(lambda: j.isSuppressed),
        "health_state": read(lambda: j.healthState),
        "health_message": read(lambda: j.errorOrWarningMessage),
    }
    if kind == "joint":
        result.update({
            "is_flipped": read(lambda: j.isFlipped),
            "geometry_one": read(lambda: geometry(j.geometryOrOriginOne)),
            "geometry_two": read(lambda: geometry(j.geometryOrOriginTwo)),
            "geometry_one_transform": read(lambda: matrix(j.geometryOneTransform)),
            "geometry_two_transform": read(lambda: matrix(j.geometryTwoTransform)),
        })
    else:
        result["geometry"] = read(lambda: geometry(j.geometry))
        result["transform"] = read(lambda: matrix(j.transform))
    if m.jointType == adsk.fusion.JointTypes.RevoluteJointType:
        result.update({
            "axis_vector": read(lambda: vec(m.rotationAxisVector)),
            "rotation_rad": read(lambda: m.rotationValue),
            "rotation_limits": read(lambda: limits(m.rotationLimits)),
        })
    return result


def pose_snapshot(root):
    return {
        "rotations_rad": {j.name: j.jointMotion.rotationValue for j in root.allJoints
                          if j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType},
        "occurrences": {o.fullPathName: matrix(o.transform2) for o in root.allOccurrences},
    }


def drive_samples(root):
    moving = [j for j in root.allJoints
              if j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType]
    original = {j.name: j.jointMotion.rotationValue for j in moving}
    normalized = {name: math.atan2(math.sin(q), math.cos(q)) for name, q in original.items()}
    result = {"baseline": pose_snapshot(root), "samples": [], "restore_errors": []}
    try:
        for j in moving:
            for delta_deg in (5.0, -5.0):
                sample = {"driven_joint": j.name, "requested_delta_deg": delta_deg}
                try:
                    j.jointMotion.rotationValue = normalized[j.name] + math.radians(delta_deg)
                    adsk.doEvents()
                    sample["pose"] = pose_snapshot(root)
                except Exception:
                    sample["error"] = traceback.format_exc()
                finally:
                    j.jointMotion.rotationValue = original[j.name]
                    adsk.doEvents()
                result["samples"].append(sample)
        sample = {"driven_joint": "all", "requested_deltas_deg": {j.name: 5.0 for j in moving}}
        for j in moving:
            j.jointMotion.rotationValue = normalized[j.name] + math.radians(5.0)
        adsk.doEvents()
        sample["pose"] = pose_snapshot(root)
        result["samples"].append(sample)
        rng = random.Random(20261019)
        # CAD-only FK checks; these poses are not certified collision-free.
        ranges_deg = {
            "base_yaw": (-70, 70), "shoulder_pitch": (-70, 70),
            "elbow_pitch": (-70, 80), "wrist_pitch": (-70, 70),
            "wrist_roll": (-90, 90), "gripper_moving": (-5, 60),
        }
        for index in range(20):
            requested = {j.name: rng.uniform(*ranges_deg[j.name]) for j in moving}
            sample = {"driven_joint": "random_combination", "index": index,
                      "random_seed": 20261019, "requested_angles_deg": requested}
            for j in moving:
                j.jointMotion.rotationValue = math.radians(requested[j.name])
            adsk.doEvents()
            sample["pose"] = pose_snapshot(root)
            result["samples"].append(sample)
    finally:
        for j in moving:
            try:
                j.jointMotion.rotationValue = original[j.name]
            except Exception as exc:
                result["restore_errors"].append({"joint": j.name, "error": str(exc)})
        adsk.doEvents()
        result["restored"] = pose_snapshot(root)
        (OUT / "fusion_pose_samples.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    OUT.mkdir(parents=True, exist_ok=True)
    try:
        baseline_source()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design or "PingTi" not in app.activeDocument.name:
            raise RuntimeError("Activate the imported PingTi F3D document first.")
        root = design.rootComponent
        data = {
            "schema": "pingti-fusion-audit/1",
            "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "fusion_version": app.version,
            "document_name": app.activeDocument.name,
            "root_component_name": root.name,
            "source_path": str(SOURCE),
            "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            "units": {"position": "m", "angle": "rad", "mass": "kg", "raw_transform_translation": "cm", "raw_inertia": "kg*cm^2"},
            "geometry_edited_by_script": False,
            "document_saved_by_script": False,
            "joint_pose_temporarily_driven_and_restored": True,
            "root_physical_properties": read(lambda: props(root)),
            "occurrences": [occurrence(o) for o in root.allOccurrences],
            "joints": [joint(j, "joint") for j in root.allJoints],
            "as_built_joints": [joint(j, "as_built") for j in root.allAsBuiltJoints],
        }
        target = OUT / "fusion_snapshot.json"
        target.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        samples = drive_samples(root)
        ui.messageBox("Audit exported; CAD file was not saved.\n{} occurrences, {} joints, {} motion samples.\nRestore errors: {}\n{}".format(
            len(data["occurrences"]), len(data["joints"]), len(samples["samples"]),
            len(samples["restore_errors"]), target), "PingTi URDF audit")
    except Exception:
        error = traceback.format_exc()
        (OUT / "fusion_error.txt").write_text(error, encoding="utf-8")
        ui.messageBox(error, "PingTi audit error")
