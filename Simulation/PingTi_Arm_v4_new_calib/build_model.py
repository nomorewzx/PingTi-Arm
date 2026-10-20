"""Build and verify the new model from the local Fusion application's export.

Run with a Python environment containing NumPy and SciPy. No motor interfaces
are imported. FusionNewCalib.py produces provenance/fusion_export.json first.
"""

import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import warnings
import xml.etree.ElementTree as ET

from source_artifacts import baseline_f3d_bytes, baseline_urdf_bytes, git_blob

import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[1]
SO = Path("/Users/zxwang/repos/SO-ARM100/Simulation/SO101")
RAW = OUT / "provenance"
OLD_URDF = REPO / "Simulation/PingTi_Arm_v3/urdf/PingTi_Arm_v3.urdf"
OLD_SHA256 = "fdb1781047c65f24d7d44042dd886784317775a60b9606de11121d5bd7e92c85"
SOURCE_SHA256 = "d593c01c38da6e048790d1c9c18e48add82de338029c5f8759322417adfaf582"
URDF = OUT / "pingti_arm_v4_new_calib.urdf"
NAMES = {"base_yaw": "shoulder_pan", "shoulder_pitch": "shoulder_lift",
         "elbow_pitch": "elbow_flex", "wrist_pitch": "wrist_flex",
         "wrist_roll": "wrist_roll", "gripper_moving": "gripper"}
# Nominal motor directions follow the SO new_calib chain: the three pitch axes
# are parallel, the roll axis points toward the forearm, and positive jaw opens.
# Their agreement with actual PingTi encoder directions is a hardware check.
SIGNS = {"base_yaw": 1, "shoulder_pitch": 1, "elbow_pitch": -1,
         "wrist_pitch": -1, "wrist_roll": -1, "gripper_moving": 1}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def transform(value):
    t = np.array(value["row_major_cm"], dtype=float).reshape(4, 4)
    t[:3, 3] *= 0.01
    return t


def unit(v):
    v = np.array(v, dtype=float)
    return v / np.linalg.norm(v)


def xyzrpy(t):
    # Euler angles are non-unique at a pitch of pi/2. SciPy's selected equivalent
    # representation is valid and is checked again through FK after serialization.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected.*")
        rpy = Rotation.from_matrix(t[:3, :3]).as_euler("xyz")
    return {"xyz": fmt(t[:3, 3]), "rpy": fmt(rpy)}


def fmt(values):
    return " ".join(format(float(x), ".14g") for x in values)


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        t[:3, :3] = Rotation.from_euler("xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return t


def triangles(path):
    b = path if isinstance(path, bytes) else path.read_bytes()
    n = struct.unpack_from("<I", b, 80)[0]
    if len(b) != 84 + 50*n:
        raise ValueError("Not a binary STL: " + str(path))
    return np.frombuffer(b, dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)["v"].astype(float)


def mesh_properties(path):
    tri = triangles(path)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    volumes = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
    volume = volumes.sum()
    summed = a+b+c
    first = (volumes[:, None] * summed / 4).sum(axis=0)
    second = np.einsum("n,ni,nj->ij", volumes/20, summed, summed)
    for v in (a, b, c):
        second += np.einsum("n,ni,nj->ij", volumes/20, v, v)
    if volume < 0:
        volume, first, second = -volume, -first, -second
    center = first/volume
    covariance = second - volume*np.outer(center, center)
    inertia_density_one = np.trace(covariance)*np.eye(3) - covariance
    vertices = tri.reshape(-1, 3)
    return {"volume_m3": float(volume), "center_m": center,
            "inertia_density_one": inertia_density_one,
            "bbox_m": np.array([vertices.min(axis=0), vertices.max(axis=0)])}


def prepare_overrides():
    """Convert user-selected STL geometry to meters without changing frames."""
    config_path = OUT / "geometry_overrides.json"
    if not config_path.exists():
        return {}, None
    config = json.loads(config_path.read_text())
    result = {}
    for occurrence, selection in config["overrides"].items():
        source = REPO / selection["source_path"]
        assert sha(source) == selection["source_sha256"], "Selected STL changed: " + str(source)
        assert selection["source_units"] == "mm"
        data = source.read_bytes()
        count = struct.unpack_from("<I", data, 80)[0]
        assert len(data) == 84 + 50*count
        dtype = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
        records = np.frombuffer(data, dtype=dtype, count=count, offset=84).copy()
        records["v"] *= .001
        target = OUT / selection["output_mesh"]
        target.write_bytes(b"PingTi strengthened elbow; SI meters; native component frame".ljust(80, b" ")+struct.pack("<I", count)+records.tobytes())
        tri = triangles(target)
        _, ids = np.unique(tri.reshape(-1, 3), axis=0, return_inverse=True)
        faces = ids.reshape(-1, 3)
        edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        _, inverse, multiplicity = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True)
        winding = np.bincount(inverse, weights=np.where(edges[:, 0] < edges[:, 1], 1., -1.))
        areas = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)/2
        assert (multiplicity == 2).all() and (winding == 0).all(), "Replacement mesh is not a consistently wound closed surface"
        assert (areas > 1e-18).all(), "Replacement mesh contains degenerate triangles"
        result[occurrence] = {
            "selection": selection,
            "mesh": {"path": selection["output_mesh"], "triangles": count, "sha256": sha(target), "units": "m"},
            "topology": {"watertight": True, "consistent_winding": True, "degenerate_triangles": 0}}
    return result, config


def fk(root, positions=None):
    positions = positions or {}
    joints = root.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    roots = [l.get("name") for l in root.findall("link") if l.get("name") not in children]
    if len(roots) != 1:
        raise ValueError("Expected one root: " + str(roots))
    transforms = {roots[0]: np.eye(4)}
    frames = {}
    pending = list(joints)
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            frame = transforms[parent] @ origin(joint.find("origin"))
            frames[joint.get("name")] = frame
            motion = np.eye(4)
            if joint.get("type") != "fixed":
                axis = unit(np.fromstring(joint.find("axis").get("xyz"), sep=" "))
                motion[:3, :3] = Rotation.from_rotvec(axis*positions.get(joint.get("name"), 0.)).as_matrix()
            transforms[joint.find("child").get("link")] = frame @ motion
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError("Joint graph is disconnected or cyclic")
    return transforms, frames


def so_reference():
    reference = ET.parse(SO / "so101_new_calib.urdf").getroot()
    links = {l.get("name"): l for l in reference.findall("link")}
    # The moving jaw has no servo. It therefore provides an identifiable
    # effective printed-part density rather than mixing servo and print mass.
    jaw_mass = float(links["moving_jaw_so101_v1_link"].find("inertial/mass").get("value"))
    jaw_mesh = SO / "assets/moving_jaw_so101_v1.stl"
    jaw_volume = mesh_properties(jaw_mesh)["volume_m3"]
    density = jaw_mass/jaw_volume
    # The fixed gripper link comprises exactly one printed body and one servo.
    gripper_mass = float(links["gripper_link"].find("inertial/mass").get("value"))
    printed_mesh = SO / "assets/wrist_roll_follower_so101_v1.stl"
    printed_volume = mesh_properties(printed_mesh)["volume_m3"]
    servo_mass = gripper_mass - density*printed_volume
    _, frames = fk(reference)
    axes = {j.get("name"): frames[j.get("name")][:3, :3] @ unit(np.fromstring(j.find("axis").get("xyz"), sep=" "))
            for j in reference.findall("joint") if j.get("type") != "fixed"}
    a, b, k = [axes[n] for n in ("wrist_flex", "gripper", "wrist_roll")]
    wrist_reference_angle = math.atan2(k @ np.cross(a, b), a @ b)
    reference_dir = RAW / "so_arm_reference"
    reference_dir.mkdir(exist_ok=True)
    files = [SO / "so101_new_calib.urdf", jaw_mesh, printed_mesh]
    evidence = []
    for path in files:
        shutil.copy2(path, reference_dir / path.name)
        evidence.append({"original_path": str(path), "saved_copy": str((reference_dir/path.name).relative_to(OUT)), "sha256": sha(path)})
    shutil.copy2(SO.parents[1] / "LICENSE", reference_dir / "LICENSE")
    commit = subprocess.check_output(["git", "-C", str(SO.parents[1]), "rev-parse", "HEAD"], text=True).strip()
    result = {
        "repository": "https://github.com/TheRobotStudio/SO-ARM100",
        "local_commit": commit, "files": evidence,
        "reference_model": "SO101/so101_new_calib.urdf",
        "effective_print_density_kg_m3": density,
        "density_derivation": {"reference_jaw_mass_kg": jaw_mass, "reference_jaw_mesh_volume_m3": jaw_volume,
                               "formula": "rho = mass of servo-free SO jaw / volume of its mesh"},
        "servo_mass_kg": servo_mass,
        "servo_mass_derivation": {"reference_gripper_link_mass_kg": gripper_mass,
                                  "reference_printed_body_mesh_volume_m3": printed_volume,
                                  "formula": "servo_mass = gripper_link_mass - rho * printed_body_volume"},
        "wrist_pitch_to_jaw_signed_angle_deg": math.degrees(wrist_reference_angle),
        "scope": "Effective density and servo mass inferred from two separable SO link assemblies, under the user's common-density assumption. Not manufacturer measurements or identification of the entire SO model.",
        "sts3250_mass": "Same inferred servo mass, explicitly applying the user's initial approximation."
    }
    return result, wrist_reference_angle


def build():
    assert sha(OLD_URDF) == OLD_SHA256, "Original URDF changed since the audit"
    assert hashlib.sha256(baseline_f3d_bytes()).hexdigest() == SOURCE_SHA256
    data = json.loads((RAW / "fusion_export.json").read_text())
    assert data["source_sha256"] == SOURCE_SHA256
    overrides, override_config = prepare_overrides()
    reference, wrist_angle = so_reference()
    zero_pose = next(p for p in data["poses"] if p["label"] == "so101_new_calib_wrist_reference")
    expected_roll_deg = 270. - math.degrees(wrist_angle)
    assert abs(zero_pose["requested_angles_deg"]["wrist_roll"] - expected_roll_deg) < 1e-8
    zero = zero_pose["snapshot"]
    joints_zero = {j["name"]: j for j in zero_pose["joints"]}
    occurrences = {o["full_path"]: o for o in data["occurrences"]}
    source_joints = data["joints"] + data["as_built_joints"]
    child_paths = {j["occurrence_one"] for j in source_joints}
    root_path, = [p for p in occurrences if p not in child_paths]
    parent_joint = {j["occurrence_one"]: j for j in source_joints}
    link_names = {p: ("base_link" if p == root_path else re.sub(r"[^a-z0-9_]+", "_", o["component_name"].lower()).strip("_"))
                  for p, o in occurrences.items()}
    assert len(set(link_names.values())) == len(link_names)
    cad_occ_zero = {p: transform(value) for p, value in zero["occurrences"].items()}
    anchors_zero = {n: np.array(j["geometry_two_transform"]["translation_m"]) for n, j in joints_zero.items()}
    axes_zero = {n: np.array(j["axis_vector"]) for n, j in joints_zero.items()}
    # This Fusion document rounds driven joint values to 0.1 degree. Retarget the
    # wrist reference analytically by the residual rigid rotation, using the
    # independently exported axis/pivot, instead of rounding the SO reference.
    actual_roll_zero = zero["rotations_rad"]["wrist_roll"]
    desired_roll_zero = math.radians(expected_roll_deg)
    residual = desired_roll_zero-actual_roll_zero
    wrist_child = next(j["occurrence_one"] for j in source_joints if j["name"] == "wrist_roll")
    descendants = {wrist_child}
    while True:
        expanded = descendants | {j["occurrence_one"] for j in source_joints if j["occurrence_two"] in descendants}
        if expanded == descendants:
            break
        descendants = expanded
    correction = np.eye(4)
    correction[:3, :3] = Rotation.from_rotvec(unit(axes_zero["wrist_roll"])*residual).as_matrix()
    pivot = anchors_zero["wrist_roll"]
    correction[:3, 3] = pivot-correction[:3, :3]@pivot
    for path in descendants:
        cad_occ_zero[path] = correction @ cad_occ_zero[path]
    for source_joint in source_joints:
        name = source_joint["name"]
        if name in anchors_zero and source_joint["occurrence_two"] in descendants:
            anchors_zero[name] = (correction @ np.r_[anchors_zero[name], 1.])[:3]
            axes_zero[name] = correction[:3, :3] @ axes_zero[name]
    # Base frame: +X points forward along the arm, +Y along the shoulder's
    # positive axis, and +Z upward; origin is the base-yaw ground projection.
    cad_from_base = np.eye(4)
    cad_from_base[:3, :3] = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
    cad_from_base[:3, 3] = anchors_zero["base_yaw"]
    cad_from_base[2, 3] = occurrences[root_path]["bbox_m"][0][2]
    frames = {root_path: cad_from_base}
    for path in occurrences:
        if path == root_path:
            continue
        joint = parent_joint[path]
        frame = cad_occ_zero[path].copy()
        if joint["motion_type"] == 1:
            frame[:3, 3] = anchors_zero[joint["name"]]
        frames[path] = frame
    root = ET.Element("robot", {"name": "pingti_arm_v4_new_calib"})
    root.append(ET.Comment("PingTi v4 kinematic chain in SI units, with user-selected geometry overrides recorded in geometry_overrides.json. Original v3 URDF is preserved."))
    mujoco = ET.SubElement(root, "mujoco")
    ET.SubElement(mujoco, "compiler", {"meshdir": "meshes", "discardvisual": "false", "fusestatic": "false"})
    for name, color in [("printed", "0.88 0.67 0.19 1"), ("servo", "0.16 0.18 0.21 1")]:
        ET.SubElement(ET.SubElement(root, "material", {"name": name}), "color", {"rgba": color})
    manifest_links = []
    for path, occurrence in occurrences.items():
        name = link_names[path]
        link = ET.SubElement(root, "link", {"name": name})
        replacement = overrides.get(path)
        mesh_metadata = replacement["mesh"] if replacement else occurrence["mesh"]
        mesh_path = OUT / mesh_metadata["path"]
        prop = occurrence["native_component_properties"]
        mesh = mesh_properties(mesh_path)
        servo = occurrence["component_name"].startswith("STS")
        if replacement:
            assert not servo and name == replacement["selection"]["link_name"]
            # Check the component coordinate envelope, not just its overall size.
            baseline_mesh = str((OUT/occurrence["mesh"]["path"]).relative_to(REPO))
            native_mesh = mesh_properties(git_blob(baseline_mesh, occurrence["mesh"]["sha256"]))
            assert np.max(np.abs(mesh["bbox_m"]-native_mesh["bbox_m"])) < 5e-5
            center = mesh["center_m"]
            # Reference inertia at uniform density 1 kg/m^3; scale below by rho.
            old_mass = mesh["volume_m3"]
            inertia_com = mesh["inertia_density_one"]
            mass = mesh["volume_m3"]*reference["effective_print_density_kg_m3"]
            inertia_error = offdiag_sign = volume_error = com_error = None
        else:
            center = np.array(prop["center_of_mass_m"])
            old_mass = prop["mass_kg"]
            values = np.array(prop["inertia_world_kg_cm2_success_xx_yy_zz_xy_yz_xz"][1:])*1e-4
            xx, yy, zz, xy, yz, xz = values
            candidates = []
            for offdiag_sign in (1, -1):
                inertia_origin = np.array([[xx, offdiag_sign*xy, offdiag_sign*xz], [offdiag_sign*xy, yy, offdiag_sign*yz], [offdiag_sign*xz, offdiag_sign*yz, zz]])
                inertia_com = inertia_origin - old_mass*(np.dot(center, center)*np.eye(3)-np.outer(center, center))
                mesh_inertia = mesh["inertia_density_one"]*old_mass/mesh["volume_m3"]
                error = np.linalg.norm(inertia_com-mesh_inertia)/np.linalg.norm(mesh_inertia)
                candidates.append((error, offdiag_sign, inertia_com))
            inertia_error, offdiag_sign, inertia_com = min(candidates, key=lambda item: item[0])
            assert inertia_error < 0.025, (path, "native inertia mismatch", inertia_error)
            com_error = float(np.linalg.norm(center-mesh["center_m"]))
            assert com_error < 0.0001, (path, "component mesh frame mismatch")
            volume_error = abs(mesh["volume_m3"]/(prop["volume_cm3"]*1e-6)-1)
            mass = reference["servo_mass_kg"] if servo else prop["volume_cm3"]*1e-6*reference["effective_print_density_kg_m3"]
        scale = mass/old_mass
        mesh_to_link = np.linalg.inv(frames[path]) @ cad_occ_zero[path]
        center_link = (mesh_to_link @ np.r_[center, 1.])[:3]
        r = mesh_to_link[:3, :3]
        inertia_link = r @ (inertia_com*scale) @ r.T
        eig = np.linalg.eigvalsh(inertia_link)
        assert min(eig) > 0 and max(eig) <= sum(eig)-max(eig)+1e-12
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": fmt(center_link), "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": format(mass, ".14g")})
        ET.SubElement(inertial, "inertia", {key: format(float(inertia_link[i, j]), ".14g") for key, i, j in
                      [("ixx", 0, 0), ("iyy", 1, 1), ("izz", 2, 2), ("ixy", 0, 1), ("ixz", 0, 2), ("iyz", 1, 2)]})
        for kind in ("visual", "collision"):
            part = ET.SubElement(link, kind, {"name": name + "_" + kind})
            ET.SubElement(part, "origin", xyzrpy(mesh_to_link))
            ET.SubElement(ET.SubElement(part, "geometry"), "mesh", {"filename": mesh_metadata["path"], "scale": "1 1 1"})
            if kind == "visual":
                ET.SubElement(part, "material", {"name": "servo" if servo else "printed"})
        manifest_links.append({"name": name, "cad_occurrence": path, "mesh": mesh_metadata,
                               "mass_kg": mass, "category": "servo" if servo else "printed",
                               "cad_volume_cm3": None if replacement else prop["volume_cm3"],
                               "mesh_volume_cm3": mesh["volume_m3"]*1e6,
                               "mass_property_source": "STL closed-surface volume integrals at SO effective density" if replacement else "Native Fusion component properties, rescaled to assigned mass",
                               "geometry_override": replacement,
                               "mesh_to_link": mesh_to_link.tolist(),
                               "inertia_eigenvalues_kg_m2": eig.tolist(),
                               "native_inertia_offdiagonal_sign": offdiag_sign,
                               "mesh_vs_native_inertia_relative_error": float(inertia_error) if inertia_error is not None else None,
                               "mesh_vs_native_volume_relative_error": volume_error,
                               "mesh_vs_native_com_error_m": com_error})
    manifest_joints = []
    for source_joint in source_joints:
        old_name = source_joint["name"]
        path = source_joint["occurrence_one"]
        parent = source_joint["occurrence_two"]
        moving = source_joint["motion_type"] == 1
        name = NAMES.get(old_name, re.sub(r"[^a-z0-9_]+", "_", old_name.lower()).strip("_"))
        joint = ET.SubElement(root, "joint", {"name": name, "type": "revolute" if moving else "fixed"})
        ET.SubElement(joint, "parent", {"link": link_names[parent]})
        ET.SubElement(joint, "child", {"link": link_names[path]})
        relative = np.linalg.inv(frames[parent]) @ frames[path]
        ET.SubElement(joint, "origin", xyzrpy(relative))
        row = {"name": name, "cad_joint": old_name, "parent": link_names[parent], "child": link_names[path], "type": joint.get("type")}
        if moving:
            sign = SIGNS[old_name]
            offset = desired_roll_zero if old_name == "wrist_roll" else zero["rotations_rad"][old_name]
            axis = unit(frames[path][:3, :3].T @ axes_zero[old_name])*sign
            ET.SubElement(joint, "axis", {"xyz": fmt(axis)})
            if old_name == "wrist_roll":
                lo, hi = -math.pi, math.pi
            else:
                limits = joints_zero[old_name]["rotation_limits"]
                lo, hi = sorted(sign*(limits[k]-offset) for k in ("minimumValue", "maximumValue"))
            # Position planning defaults, deliberately distinct from the v3
            # exporter's generic 100 values. These are not driver protections.
            effort = 6.70 if name in ("shoulder_lift", "elbow_flex") else 3.35
            ET.SubElement(joint, "limit", {"lower": format(lo, ".14g"), "upper": format(hi, ".14g"), "effort": str(effort), "velocity": "1.0"})
            ET.SubElement(joint, "dynamics", {"damping": "0.6", "friction": "0.052"})
            row.update({"cad_angle_at_urdf_zero_deg": math.degrees(offset), "cad_to_urdf_sign": sign,
                        "cad_to_urdf_formula": "q_urdf_rad = sign * (q_cad_rad - zero_cad_rad)",
                        "limits_deg": [math.degrees(lo), math.degrees(hi)], "axis": axis.tolist()})
        manifest_joints.append(row)
    # A named output-horn frame is useful for attaching a separately measured
    # TCP. It is deliberately not mislabeled as a calibrated grasp point.
    mount_link = ET.SubElement(root, "link", {"name": "tool_mount_link"})
    mount_inertial = ET.SubElement(mount_link, "inertial")
    ET.SubElement(mount_inertial, "mass", {"value": "0"})
    ET.SubElement(mount_inertial, "inertia", {key: "0" for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")})
    mount = ET.SubElement(root, "joint", {"name": "tool_mount_joint", "type": "fixed"})
    roll_row = next(row for row in manifest_joints if row["name"] == "wrist_roll")
    ET.SubElement(mount, "parent", {"link": roll_row["child"]})
    ET.SubElement(mount, "child", {"link": "tool_mount_link"})
    ET.SubElement(mount, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(URDF, encoding="utf-8", xml_declaration=True)
    manifest = {"schema": "pingti-new-calib-model/1", "urdf": URDF.name, "urdf_sha256": sha(URDF),
                "source_f3d_sha256": SOURCE_SHA256, "source_f3d_path": "fusion/PingTi_Arm_5DOF_v4.f3d",
                "source_f3d_commit": override_config["baseline_commit"],
                "original_urdf_sha256": OLD_SHA256,
                "fusion_version": data["fusion_version"], "cad_from_base": cad_from_base.tolist(),
                "reference": reference, "links": manifest_links, "joints": manifest_joints,
                "geometry_overrides": override_config,
                "mass_total_kg": sum(l["mass_kg"] for l in manifest_links),
                "nominal_direction_source": "SO new_calib pitch axes and wrist reference geometry; actual motor encoder directions have not been measured.",
                "wrist_reference": "Match SO101 new_calib signed pitch-to-jaw angle, not its numerical origin rpy. The corresponding PingTi CAD wrist angle is derived from the copied mechanism.",
                "fusion_reference_rounding": {"actual_exported_roll_deg": math.degrees(actual_roll_zero), "desired_roll_deg": expected_roll_deg, "analytic_residual_rotation_deg": math.degrees(residual), "method": "Rigid rotation of the downstream subtree about the measured CAD wrist pivot and axis; no sample fitting."},
                "planning_limits": "velocity=1 rad/s; effort=3.35 Nm per servo or 6.70 for paired joints, using SO motor reference. Approximate model values, not verified PingTi limits or driver enforcement.",
                "tool_mount": "Output-horn frame; not a calibrated gripper TCP.",
                "collision_geometry": "Native CAD triangulations plus the selected full-length strengthened elbow STL. Simulators may convexify individual meshes; unchanged hard stops or collision-free workspace are not certified by the FK tests.",
                "hardware_validation": "Not performed; confirm signs, repeatable endpoints, wrist reference and TCP before real-world Cartesian operation."}
    assembly_path = REPO / "fusion/assembly_manifest.json"
    if assembly_path.exists():
        assembly = json.loads(assembly_path.read_text())
        assert assembly["status"] == "passed"
        assert sha(REPO/assembly["f3d"]) == assembly["f3d_sha256"]
        assert sha(REPO/assembly["validation_report"]) == assembly["validation_sha256"]
        assert assembly.get("reference_stl_sha256", assembly["replacement_sha256"]) == override_config["overrides"]["elbow_link:1"]["source_sha256"]
        if "reference_stl_sha256" in assembly:
            assert sha(REPO/assembly["replacement_source"]) == assembly["replacement_sha256"]
            assert sha(REPO/assembly["geometry_match_report"]) == assembly["geometry_match_sha256"]
            shutil.copy2(REPO/assembly["geometry_match_report"], RAW/"native_geometry_match.json")
        shutil.copy2(assembly_path, RAW/"assembly_manifest.json")
        shutil.copy2(REPO/assembly["validation_report"], RAW/"fusion_assembly_validation.json")
        manifest["assembled_f3d"] = assembly
    dump(OUT / "model_manifest.json", manifest)
    return manifest, data


def validate(manifest, data):
    root = ET.parse(URDF).getroot()
    base_from_cad = np.linalg.inv(np.array(manifest["cad_from_base"]))
    errors = []
    moving = [j for j in manifest["joints"] if j["type"] != "fixed"]
    for pose in data["poses"]:
        angles = pose["snapshot"]["rotations_rad"]
        q = {j["name"]: j["cad_to_urdf_sign"]*(angles[j["cad_joint"]]-math.radians(j["cad_angle_at_urdf_zero_deg"])) for j in moving}
        q["wrist_roll"] = math.atan2(math.sin(q["wrist_roll"]), math.cos(q["wrist_roll"]))
        for joint in moving:
            lo, hi = np.radians(joint["limits_deg"])
            assert lo-1e-10 <= q[joint["name"]] <= hi+1e-10
        actual, _ = fk(root, q)
        max_point_error = max_angle_error = 0.
        for link in manifest["links"]:
            predicted = actual[link["name"]] @ np.array(link["mesh_to_link"])
            expected = base_from_cad @ transform(pose["snapshot"]["occurrences"][link["cad_occurrence"]])
            # Origin and three independent unit axes verify the complete rigid transform.
            points = np.array([[0., 0., 0., 1.], [.1, 0., 0., 1.], [0., .1, 0., 1.], [0., 0., .1, 1.]])
            max_point_error = max(max_point_error, float(np.linalg.norm((points @ predicted.T-points @ expected.T)[:, :3], axis=1).max()))
            angle_error = Rotation.from_matrix(predicted[:3, :3].T @ expected[:3, :3]).magnitude()
            max_angle_error = max(max_angle_error, float(angle_error))
        errors.append({"pose": pose["label"], "max_point_error_mm": max_point_error*1000,
                       "max_orientation_error_deg": math.degrees(max_angle_error)})
    worst_point = max(e["max_point_error_mm"] for e in errors)
    worst_angle = max(e["max_orientation_error_deg"] for e in errors)
    assert worst_point < 1e-5 and worst_angle < 1e-5, (worst_point, worst_angle)
    # Confirm the intended nominal SO wrist reference from the exported model.
    _, frames = fk(root)
    joints = {j.get("name"): j for j in root.findall("joint")}
    axes = {n: unit(frames[n][:3, :3] @ np.fromstring(joints[n].find("axis").get("xyz"), sep=" "))
            for n in ("shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
    angle = math.degrees(math.atan2(axes["wrist_roll"] @ np.cross(axes["wrist_flex"], axes["gripper"]), axes["wrist_flex"] @ axes["gripper"]))
    assert abs(angle-manifest["reference"]["wrist_pitch_to_jaw_signed_angle_deg"]) < 1e-6
    assert axes["shoulder_lift"] @ axes["elbow_flex"] > .999999
    assert axes["shoulder_lift"] @ axes["wrist_flex"] > .999999
    restore_error = max(np.max(np.abs(transform(data["baseline"]["occurrences"][p])-transform(data["restored"]["occurrences"][p]))) for p in data["baseline"]["occurrences"])
    assert restore_error < 1e-9
    assert sha(OLD_URDF) == OLD_SHA256
    assert hashlib.sha256(baseline_f3d_bytes()).hexdigest() == SOURCE_SHA256
    baseline = ET.fromstring(baseline_urdf_bytes(manifest["geometry_overrides"]))
    assert [ET.tostring(j) for j in root.findall("joint")] == [ET.tostring(j) for j in baseline.findall("joint")], "Joint definitions changed"
    old_links = {l.get("name"): l for l in baseline.findall("link")}
    for link in root.findall("link"):
        if link.get("name") != "elbow_link":
            assert ET.tostring(link) == ET.tostring(old_links[link.get("name")]), "Unrelated link changed"
    replacement_links = [l for l in manifest["links"] if l["geometry_override"]]
    assert [l["name"] for l in replacement_links] == ["elbow_link"]
    native_links = [l for l in manifest["links"] if not l["geometry_override"]]
    result = {"schema": "pingti-new-calib-validation/1", "status": "passed",
              "scope": "Original Fusion kinematic chain, unchanged baseline joints, selected replacement STL, and inertial consistency; not physical endpoint, encoder or contact-task validation.",
              "physical_links": len(manifest["links"]), "moving_joints": len(moving),
              "independent_fusion_poses": len(errors), "max_fk_point_error_mm": worst_point,
              "max_fk_orientation_error_deg": worst_angle,
              "so_wrist_reference_angle_deg": angle,
              "all_inertias_positive_definite_and_triangle_inequality": True,
              "native_component_meshes_compared": len(native_links),
              "max_native_mesh_volume_relative_error": max(l["mesh_vs_native_volume_relative_error"] for l in native_links),
              "max_native_mesh_com_error_mm": 1000*max(l["mesh_vs_native_com_error_m"] for l in native_links),
              "baseline_joint_definitions_unchanged": True,
              "unrelated_link_definitions_unchanged": True,
              "replacement_mesh_topology": replacement_links[0]["geometry_override"]["topology"],
              "mass_total_kg": manifest["mass_total_kg"],
              "temporary_cad_pose_restored_max_transform_difference": float(restore_error),
              "source_f3d_preserved": True, "original_urdf_preserved": True,
              "pose_results": errors}
    dump(OUT / "validation.json", result)
    print(json.dumps({k:v for k,v in result.items() if k != "pose_results"}, indent=2))


if __name__ == "__main__":
    model, source = build()
    validate(model, source)
