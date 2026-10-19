"""Run inside the local Fusion application, using the installed Autodesk API."""

import datetime
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import struct
import traceback

import adsk.core
import adsk.fusion

HERE = Path(__file__).resolve().parent
AUDIT_DIR = HERE.parent
REPO = AUDIT_DIR.parents[2]
OUT = REPO / "Simulation/PingTi_Arm_v4_new_calib"
RAW = OUT / "provenance"
SOURCE = REPO / "fusion/PingTi_Arm_5DOF_v4.f3d"


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def triangles(body):
    calculator = body.meshManager.createMeshCalculator()
    calculator.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
    mesh = calculator.calculate()
    if not mesh:
        raise RuntimeError("No mesh for " + body.name)
    nodes = mesh.nodeCoordinatesAsDouble
    indices = mesh.nodeIndices
    for k in range(0, len(indices), 3):
        yield [[nodes[3 * i + d] * 0.01 for d in range(3)] for i in indices[k:k + 3]]


def export_mesh(component, path):
    # Native component bodies are expressed in component coordinates. Explicitly
    # convert Fusion centimeters to SI meters rather than relying on STL settings.
    faces = [face for body in component.bRepBodies for face in triangles(body)]
    with path.open("wb") as stream:
        stream.write(b"PingTi Fusion native component mesh; vertices in meters".ljust(80, b" "))
        stream.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            size = math.sqrt(sum(x*x for x in normal))
            normal = [x / size for x in normal] if size else [0., 0., 0.]
            stream.write(struct.pack("<12fH", *(normal + a + b + c), 0))
    return {"path": str(path.relative_to(OUT)), "triangles": len(faces),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "units": "m"}


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    RAW.mkdir(parents=True, exist_ok=True)
    (OUT / "meshes").mkdir(exist_ok=True)
    previous_document = app.activeDocument
    document = None
    initial = None
    audit = None
    try:
        spec = importlib.util.spec_from_file_location("pingti_fusion_audit", AUDIT_DIR / "FusionAudit/FusionAudit.py")
        audit = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(audit)
        document = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(SOURCE)))
        if document is None:
            raise RuntimeError("Could not import source F3D into a separate document")
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        initial = audit.pose_snapshot(root)
        moving = {j.name: j for j in root.allJoints
                  if j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType}
        data = {
            "schema": "pingti-native-fusion-export/1",
            "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "fusion_version": app.version,
            "source_path": str(SOURCE),
            "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            "document_name": document.name,
            "separate_fresh_import": True,
            "source_document_saved": False,
            "source_file_modified": False,
            "units": {"position": "m", "angle": "rad", "mass": "kg", "raw_transform_translation": "cm", "raw_inertia": "kg*cm^2"},
            "baseline": initial,
            "joints": [audit.joint(j, "joint") for j in root.allJoints],
            "as_built_joints": [audit.joint(j, "as_built") for j in root.allAsBuiltJoints],
            "occurrences": [],
            "poses": [],
        }
        for index, occurrence in enumerate(root.allOccurrences):
            item = audit.occurrence(occurrence)
            item["mesh"] = export_mesh(occurrence.component, OUT / "meshes" / ("component_%02d.stl" % index))
            item["native_component_properties"] = audit.read(lambda: audit.props(occurrence.component))
            data["occurrences"].append(item)
        # Independent Fusion solver samples, including nominal new zero,
        # mechanical endpoints, per-joint perturbations and held-out combinations.
        zero = {"base_yaw": 0., "shoulder_pitch": -5., "elbow_pitch": 10.,
                "wrist_pitch": 0., "wrist_roll": 0., "gripper_moving": -12.}
        requests = [("nominal_midpoint_roll_plane", zero.copy())]
        for roll in [2.78912989965, -2.78912989965, 90., -90.]:
            q = zero.copy(); q["wrist_roll"] = roll
            requests.append(("roll_reference_%+.9f" % roll, q))
        q = zero.copy(); q["wrist_roll"] = 92.78912989885248
        requests.append(("so101_new_calib_wrist_reference", q))
        ranges = {"base_yaw": (-110., 110.), "shoulder_pitch": (-110., 100.),
                  "elbow_pitch": (-90., 110.), "wrist_pitch": (-110., 110.),
                  "wrist_roll": (-179., 179.), "gripper_moving": (-12., 90.)}
        for name, (lo, hi) in ranges.items():
            for value in (lo, hi, zero[name] + 5.):
                q = zero.copy(); q[name] = value
                requests.append(("single_%s_%+.6f" % (name, value), q))
        rng = random.Random(20261019)
        for index in range(48):
            q = {name: rng.uniform(lo, hi) for name, (lo, hi) in ranges.items()}
            requests.append(("random_%02d" % index, q))
        write_json(RAW / "fusion_export.json", data)
        for label, requested in requests:
            for name, angle in requested.items():
                moving[name].jointMotion.rotationValue = math.radians(angle)
            adsk.doEvents()
            data["poses"].append({"label": label, "requested_angles_deg": requested,
                                  "snapshot": audit.pose_snapshot(root),
                                  "joints": [audit.joint(j, "joint") for j in moving.values()]})
        for name, angle in initial["rotations_rad"].items():
            moving[name].jointMotion.rotationValue = angle
        adsk.doEvents()
        data["restored"] = audit.pose_snapshot(root)
        write_json(RAW / "fusion_export.json", data)
        ui.messageBox("PingTi native export completed.\n%d meshes; %d independent CAD poses.\nOriginal F3D and URDF were not saved or edited.\n%s" % (
            len(data["occurrences"]), len(data["poses"]), OUT), "PingTi new_calib export")
    except Exception:
        error = traceback.format_exc()
        (RAW / "fusion_error.txt").write_text(error, encoding="utf-8")
        ui.messageBox(error, "PingTi new_calib export error")
    finally:
        if document:
            # Only discard the temporary document created by this script.
            document.close(False)
        if previous_document:
            previous_document.activate()
