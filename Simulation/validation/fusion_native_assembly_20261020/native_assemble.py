"""Run using Fusion's native Scripts and Add-Ins UI; work on a fresh import."""
import datetime
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import traceback

import adsk.core
import adsk.fusion

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[2]
AUDIT = REPO / "Simulation/validation/urdf_audit_20261019/FusionAudit/FusionAudit.py"
ARCHIVE = REPO / "fusion/archive/PingTi_Arm_5DOF_v4_original.f3d"
SOURCE = ARCHIVE
STL = REPO / "stl_3d_printing/elbow_link.stl"
CANDIDATE = OUT / "PingTi_Arm_5DOF_v4_native_candidate.f3d"
NATIVE = REPO / "fusion/parts/shoulder_to_elbow_link_v2_native_source.f3d"
NATIVE_VOLUME = 346.4837601164948
POSES = REPO / "Simulation/PingTi_Arm_v4_new_calib/provenance/fusion_export.json"
OLD_SHA = "d593c01c38da6e048790d1c9c18e48add82de338029c5f8759322417adfaf582"
STL_SHA = "657d46991fd8fa4cb5acb3c49868b154eecef5a67a66eea08b72f84aca0a548d"


def dump(data):
    (OUT / "assembly_validation.json").write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def compare_pose(expected, actual):
    assert expected["occurrences"].keys() == actual["occurrences"].keys()
    rotation_error = translation_error_cm = 0.
    for name, item in expected["occurrences"].items():
        a = item["row_major_cm"]
        b = actual["occurrences"][name]["row_major_cm"]
        rotation_error = max(rotation_error, *(abs(a[i]-b[i]) for i in (0, 1, 2, 4, 5, 6, 8, 9, 10)))
        translation_error_cm = max(translation_error_cm, *(abs(a[i]-b[i]) for i in (3, 7, 11)))
    assert rotation_error < 1e-7, rotation_error
    assert translation_error_cm < 1e-6, translation_error_cm
    return {"max_rotation_matrix_element_error": rotation_error,
            "max_translation_coordinate_error_mm": translation_error_cm * 10.}


def joint_records(root, audit):
    return [joint_record(j, audit) for j in list(root.allJoints) + list(root.allAsBuiltJoints)]


def joint_record(joint, audit):
    regular = joint.objectType == adsk.fusion.Joint.classType()
    record = audit.joint(joint, "joint" if regular else "as_built")
    if not regular:
        timeline = joint.timelineObject
        record["health_state"] = timeline.healthState if timeline else None
        record["health_message"] = timeline.errorOrWarningMessage if timeline else ""
        record["health_source"] = "timelineObject" if timeline else "No timeline in direct design; references and driven poses checked explicitly"
    return record


def snapshot(root, audit):
    result = audit.pose_snapshot(root)
    result["rotations_rad"].update({j.name: j.jointMotion.rotationValue for j in root.allAsBuiltJoints
                                   if j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType})
    return result


def validate_assembly(root, audit, reference, original_occurrences):
    assert root.allOccurrences.count == 16
    assert len(list(root.allJoints)) + len(list(root.allAsBuiltJoints)) == 15
    joints = {j.name: j for j in list(root.allJoints) + list(root.allAsBuiltJoints)}
    expected = {j["name"]: j for j in reference}
    assert joints.keys() == expected.keys()
    checks = []
    for name, joint in joints.items():
        is_regular = joint.objectType == adsk.fusion.Joint.classType()
        record = joint_record(joint, audit)
        for field in ("motion_type", "suppressed"):
            assert record[field] == expected[name][field], (name, field)
        assert {record["occurrence_one"], record["occurrence_two"]} == {expected[name]["occurrence_one"], expected[name]["occurrence_two"]}, name
        if is_regular and name not in ("elbow_pitch", "elbow_wrist_servo_connect", "wrist_mount_connect"):
            assert record["is_flipped"] == expected[name]["is_flipped"], name
        assert not record["suppressed"]
        # A direct-model design reports Unknown (5), including the original.
        # Explicitly inspect references and drive the solver instead of treating
        # Unknown as evidence of healthy motion.
        assert record["health_state"] not in (1, 2, 3, 4)
        assert record["health_message"] == ""
        if is_regular:
            for side in ("geometryOrOriginOne", "geometryOrOriginTwo"):
                geometry = getattr(joint, side)
                assert geometry and geometry.isValid
                if geometry.objectType == adsk.fusion.JointOrigin.classType():
                    geometry = geometry.geometry
                assert geometry and geometry.entityOne and geometry.entityOne.isValid, (name, side)
        else:
            assert joint.occurrenceOne.isValid and joint.occurrenceTwo.isValid
            if joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType:
                geometry = joint.geometry
                assert geometry and geometry.entityOne and geometry.entityOne.isValid
        if "rotation_limits" in record:
            assert record["rotation_limits"] == expected[name]["rotation_limits"], name
        checks.append(record)
    old = {o["full_path"]: o for o in original_occurrences}
    unchanged = []
    physical_checks = []
    for occurrence in root.allOccurrences:
        assert occurrence.component.bRepBodies.count == len(old[occurrence.fullPathName]["bodies"]), occurrence.name
        assert occurrence.component.meshBodies.count == 0, occurrence.name
        assert all(body.isSolid for body in occurrence.component.bRepBodies)
        if occurrence.component.name == "elbow_link":
            continue
        before, after = old[occurrence.fullPathName], audit.occurrence(occurrence)
        physical_checks.append({"name": occurrence.fullPathName, "before": before["physical_properties"], "after": after["physical_properties"]})
        (OUT/"native_properties_checks.json").write_text(json.dumps(physical_checks, indent=2))
        for key in ("volume_cm3", "mass_kg"):
            assert abs(before["physical_properties"][key]-after["physical_properties"][key]) < max(1e-7, abs(before["physical_properties"][key])*1e-5), (occurrence.name, key, before["physical_properties"][key], after["physical_properties"][key])
        for a, b in zip(before["physical_properties"]["center_of_mass_m"], after["physical_properties"]["center_of_mass_m"]):
            assert abs(a-b) < 1e-6, (occurrence.name, a, b)
        unchanged.append(occurrence.fullPathName)
    return {"joints": checks, "unchanged_geometry_occurrences": unchanged,
            "solid_body_count": sum(o.component.bRepBodies.count for o in root.allOccurrences), "mesh_body_count": 0}


def drive_poses(root, audit, baseline):
    moving = {j.name: j for j in list(root.allJoints) + list(root.allAsBuiltJoints)
              if j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType}
    results = []
    for pose in baseline["poses"]:
        desired = pose["snapshot"]["rotations_rad"]
        for attempt in range(3):
            for name in ("base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll", "gripper_moving"):
                moving[name].jointMotion.rotationValue = desired[name]
                adsk.doEvents()
            if all(abs(math.sin(moving[name].jointMotion.rotationValue - angle)) < 1e-7 for name, angle in desired.items()):
                break
        actual = snapshot(root, audit)
        (OUT / "pose_progress.json").write_text(json.dumps({"label": pose["label"], "expected": pose["snapshot"], "actual": actual}, indent=2))
        error = compare_pose(pose["snapshot"], actual)
        for name, angle in pose["snapshot"]["rotations_rad"].items():
            assert abs(math.sin(actual["rotations_rad"][name]-angle)) < 1e-7
        results.append({"label": pose["label"], **error})
    for name, angle in baseline["baseline"]["rotations_rad"].items():
        moving[name].jointMotion.rotationValue = angle
    adsk.doEvents()
    return results


def run(context):
    app = adsk.core.Application.get()
    previous = app.activeDocument
    document = reopened = None
    source_spec = importlib.util.spec_from_file_location("pingti_git_sources", REPO / "Simulation/PingTi_Arm_v4_new_calib/source_artifacts.py")
    sources = importlib.util.module_from_spec(source_spec)
    source_spec.loader.exec_module(sources)
    sources.ensure_baseline_f3d()
    data = {"schema": "pingti-native-strengthened-fusion-assembly/1", "status": "started",
            "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "fusion_version": app.version, "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            "stl_sha256": hashlib.sha256(STL.read_bytes()).hexdigest()}
    try:
        assert data["source_sha256"] == OLD_SHA
        assert data["stl_sha256"] == STL_SHA
        spec = importlib.util.spec_from_file_location("pingti_assembly_audit", AUDIT)
        audit = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(audit)
        # The retained manifest identifies the accepted native source. Rebuilding
        # does not require the exploratory cloud-history search records.
        source_manifest = json.loads((REPO / "fusion/assembly_manifest.json").read_text())
        assert source_manifest["status"] == "passed"
        source_version = source_manifest["native_source_version"]
        assert source_version["name"] == "shoulder_to_elbow_link" and source_version["version"] == 2
        assert source_manifest["replacement_source"] == str(NATIVE.relative_to(REPO))
        assert hashlib.sha256(NATIVE.read_bytes()).hexdigest() == source_manifest["replacement_sha256"]
        assert json.loads((OUT / "geometry_match.json").read_text())["status"] == "passed"
        native_document = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(NATIVE)))
        try:
            native_design = adsk.fusion.Design.cast(app.activeProduct)
            native_component, = [o.component for o in native_design.rootComponent.allOccurrences if o.component.name == "elbow_link"]
            native_body, = [b for b in native_component.bRepBodies if b.name == "Body1"]
            assert native_body.isVisible and native_body.isSolid and native_body.faces.count == 310
            assert abs(audit.props(native_body)["volume_cm3"] - NATIVE_VOLUME) < 1e-5
            data["native_source"] = {"file": str(NATIVE.relative_to(REPO)), "sha256": source_manifest["replacement_sha256"],
                "cloud_file": source_version, "component": "elbow_link", "body": "Body1", "body_face_count": native_body.faces.count,
                "properties": audit.props(native_body), "excluded_body": "Body3 (hidden interior solid)",
                "source_design_type": native_design.designType, "source_timeline_count": native_design.timeline.count}
            native_copy = adsk.fusion.TemporaryBRepManager.get().copy(native_body)
            assert native_copy and native_copy.isSolid
        finally:
            native_document.close(False)
        document = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(SOURCE)))
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        data["design_type"] = design.designType
        data["source_contact_analysis_enabled"] = design.isContactAnalysisEnabled
        data["source_contact_set_count"] = design.contactSets.count
        data["original_pose"] = audit.pose_snapshot(root)
        data["original_joints"] = [audit.joint(j, "joint") for j in root.allJoints]
        data["original_occurrences"] = [audit.occurrence(o) for o in root.allOccurrences]
        base_occurrence, = [o for o in root.allOccurrences if o.component.name.startswith("base_link")]
        base_occurrence.isGrounded = True
        data["base_grounded_at_original_placement"] = True
        occurrence, = [o for o in root.allOccurrences if o.component.name == "elbow_link"]
        component = occurrence.component
        bodies = list(component.bRepBodies)
        assert len(bodies) == 1
        data["joint_references"] = []
        anchors = []
        for j in root.allJoints:
            for key in ("geometryOrOriginOne", "geometryOrOriginTwo"):
                g = getattr(j, key)
                if g and g.objectType == adsk.fusion.JointGeometry.classType():
                    data["joint_references"].append({"joint": j.name, "side": key,
                        "entity_one_type": g.entityOne.objectType if g.entityOne else None,
                        "entity_two_type": g.entityTwo.objectType if g.entityTwo else None})
            anchors.append((j, "geometryOrOriginOne", j.geometryOneTransform.copy()))
        dump(data)
        # Rebuild affected constraints as as-built joints. Their attachment does
        # not depend on the old body faces, or on setters for direct joints.
        data["rebuilt_joints"] = []
        connections = []
        inverse = occurrence.transform2.copy()
        inverse.invert()
        for j, side, world in anchors:
            name, one, two = j.name, j.occurrenceOne, j.occurrenceTwo
            is_revolute = j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType
            geometry = axis_proxy = None
            limits = None
            if is_revolute:
                assert abs(math.sin(j.jointMotion.rotationValue)) < 1e-9
                limits = audit.limits(j.jointMotion.rotationLimits)
                point = adsk.core.Point3D.create(*world.translation.asArray())
                vector = j.jointMotion.rotationAxisVector.copy()
                # As-built input expects geometry in the root coordinate
                # system; component proxies are interpreted natively by this
                # Fusion API version. Independent root datum geometry supplies
                # the original global pivot and directed axis explicitly.
                point_input = root.constructionPoints.createInput()
                assert point_input.setByPoint(point)
                anchor = root.constructionPoints.add(point_input)
                anchor.name = name + "_stable_anchor"
                anchor.isLightBulbOn = False
                axis_input = root.constructionAxes.createInput()
                assert axis_input.setByLine(adsk.core.InfiniteLine3D.create(point, vector))
                axis = root.constructionAxes.add(axis_input)
                axis.name = name + "_stable_axis"
                axis.isLightBulbOn = False
                plane_input = root.constructionPlanes.createInput()
                assert plane_input.setByPlane(adsk.core.Plane.create(point, vector))
                plane = root.constructionPlanes.add(plane_input)
                plane.name = name + "_directed_datum_plane"
                plane.transform = world
                plane.isLightBulbOn = False
                sketch = root.sketches.add(plane)
                sketch.name = name + "_directed_pivot_reference"
                circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch.modelToSketchSpace(point), .1)
                circle.isConstruction = True
                sketch.isLightBulbOn = False
                geometry = adsk.fusion.JointGeometry.createByCurve(circle, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
                axis_proxy = None
                data.setdefault("directed_pivots", {})[name] = {"geometry": audit.geometry(geometry),
                    "requested_axis": audit.vec(vector), "plane_transform": list(plane.transform.asArray())}
                dump(data)
                alignment = geometry.primaryAxisVector.dotProduct(vector)
                assert abs(alignment) > .999999, data["directed_pivots"][name]
                if alignment < 0:
                    one, two = two, one
            before = snapshot(root, audit)
            assert j.deleteMe()
            connections.append((name, one, two, geometry, axis_proxy, limits))
            adsk.doEvents()
            data["rebuilt_joints"].append({"joint": name, "motion": "revolute" if is_revolute else "rigid",
                "method": "As-built constraint preserving original placement, with independent construction point/axis for the elbow",
                "pose_error": compare_pose(before, snapshot(root, audit))})
        data["reanchored_pose"] = snapshot(root, audit)
        dump(data)
        assert len(anchors) == 15
        for name, original in data["original_pose"]["occurrences"].items():
            assert max(abs(a-b) for a,b in zip(original["row_major_cm"], data["reanchored_pose"]["occurrences"][name]["row_major_cm"])) < 1e-7, name
        # Copy the persisted analytic BRep from the historical source. No STL
        # import, tessellation conversion, face fitting or mesh reduction occurs.
        assert design.designType == adsk.fusion.DesignTypes.DirectDesignType
        new_body = component.bRepBodies.add(native_copy)
        assert new_body
        new_body.name = "strong_elbow_link_native_v2"
        new_body.appearance = bodies[0].appearance
        new_body.material = bodies[0].material
        data["new_body"] = {"is_solid": new_body.isSolid, "face_count": new_body.faces.count,
            "properties": audit.props(new_body),
            "bbox_m": [audit.vec(new_body.boundingBox.minPoint, .01), audit.vec(new_body.boundingBox.maxPoint, .01)],
            "geometry_type": "Native analytic BRep copied directly from historical elbow_link Body1; not mesh converted"}
        assert new_body.isSolid and new_body.faces.count == 310
        assert abs(data["new_body"]["properties"]["volume_cm3"] - NATIVE_VOLUME) < 1e-5
        data["old_body_deleted"] = bodies[0].deleteMe()
        for m in list(component.meshBodies):
            m.deleteMe()
        adsk.doEvents()
        # As-built constraints are available only with captured design history.
        # Create the geometry and stable elbow axis in direct mode first, then
        # capture the unchanged assembly and add the two rigid constraints.
        before_history = snapshot(root, audit)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        data["capture_history_pose_check"] = compare_pose(before_history, snapshot(root, audit))
        for name, one, two, geometry, axis_proxy, limits in connections:
            joint_input = root.asBuiltJoints.createInput(one, two, geometry)
            if geometry:
                assert joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            else:
                joint_input.setAsRigidJointMotion()
            new_joint = root.asBuiltJoints.add(joint_input)
            new_joint.name = name
            new_joint.isLightBulbOn = False
            if limits:
                output = new_joint.jointMotion.rotationLimits
                for key in ("minimumValue", "maximumValue", "restValue", "isMinimumValueEnabled", "isMaximumValueEnabled", "isRestValueEnabled"):
                    setattr(output, key, limits[key])
        adsk.doEvents()
        data["final_design_type"] = design.designType
        data["after_delete_joints"] = joint_records(root, audit)
        data["after_delete_pose"] = snapshot(root, audit)
        dump(data)
        assert len(data["after_delete_joints"]) == 15
        assert len(list(component.bRepBodies)) == 1
        assert component.meshBodies.count == 0
        assert all(j["health_state"] not in (1, 2, 3, 4) and not j["health_message"] for j in data["after_delete_joints"])
        assert all("error" not in j[side] for j in data["after_delete_joints"] if j["kind"] == "joint" for side in ("geometry_one", "geometry_two"))
        baseline = json.loads(POSES.read_text())
        data["before_export_validation"] = validate_assembly(root, audit, data["original_joints"], data["original_occurrences"])
        data["before_export_poses"] = drive_poses(root, audit, baseline)
        data["before_export_restored"] = compare_pose(data["original_pose"], snapshot(root, audit))
        root.attributes.add("PingTiProvenance", "replacement_source", str(NATIVE.relative_to(REPO)))
        root.attributes.add("PingTiProvenance", "replacement_sha256", data["native_source"]["sha256"])
        root.attributes.add("PingTiProvenance", "original_f3d_sha256", OLD_SHA)
        root.attributes.add("PingTiProvenance", "geometry_type", "Native analytic BRep from shoulder_to_elbow_link cloud version 2, elbow_link Body1; copied as a base solid, with full source design archive retained separately")
        assert design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(CANDIDATE)))
        data["candidate_sha256"] = hashlib.sha256(CANDIDATE.read_bytes()).hexdigest()
        data["candidate_bytes"] = CANDIDATE.stat().st_size
        dump(data)
        reopened = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(CANDIDATE)))
        reopened_root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
        data["reopened_pose_check"] = compare_pose(data["original_pose"], snapshot(reopened_root, audit))
        data["reopened_validation"] = validate_assembly(reopened_root, audit, data["original_joints"], data["original_occurrences"])
        data["reopened_poses"] = drive_poses(reopened_root, audit, baseline)
        new_occ, = [o for o in reopened_root.allOccurrences if o.component.name == "elbow_link"]
        data["reopened_elbow_properties"] = audit.props(new_occ.component)
        assert new_occ.component.bRepBodies.item(0).faces.count == 310
        data["reopened_native_face_count"] = new_occ.component.bRepBodies.item(0).faces.count
        assert abs(data["reopened_elbow_properties"]["volume_cm3"] - NATIVE_VOLUME) < 1e-5
        app.activeViewport.fit()
        app.activeViewport.saveAsImageFile(str(OUT / "fusion_strengthened_assembly.png"), 1600, 1200)
        data["status"] = "passed"
        data["native_materials_note"] = "Original Fusion materials remain Steel. Native CAD mass is not the SO-ARM-equivalent print/servo mass used by the new_calib URDF."
        data["validation_scope"] = "Geometry, retained joint frames/limits, valid joint references, and 72 actual Fusion solver poses before and after F3D round trip; not physical clearance or hardware validation."
        dump(data)
        app.userInterface.messageBox("Native BRep F3D exported and independently reopened.\n16 components, 18 solid bodies, 15 joints, 6 moving joints.\n72 solver poses passed before and after export.\nOriginal file has not been overwritten.\n" + str(CANDIDATE), "PingTi assembly validation passed")
    except Exception:
        data["status"] = "failed"
        data["error"] = traceback.format_exc()
        dump(data)
        app.userInterface.messageBox(data["error"], "PingTi assembly probe error")
    finally:
        if reopened:
            reopened.close(False)
        if document:
            document.close(False)
        if previous:
            previous.activate()
