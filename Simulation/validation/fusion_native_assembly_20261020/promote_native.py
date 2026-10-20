"""Promote only the native candidate that passed the Fusion round-trip audit."""
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[2]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    report = OUT / "assembly_validation.json"
    data = json.loads(report.read_text())
    match = json.loads((OUT / "geometry_match.json").read_text())
    assert data["status"] == match["status"] == "passed"
    assert len(data["before_export_poses"]) == len(data["reopened_poses"]) == 72
    assert data["reopened_native_face_count"] == 310
    assert len(data["reopened_validation"]["joints"]) == 15
    assert all(j["health_state"] == 0 for j in data["reopened_validation"]["joints"])
    candidate = OUT / "PingTi_Arm_5DOF_v4_native_candidate.f3d"
    assert sha(candidate) == data["candidate_sha256"]
    native = REPO / data["native_source"]["file"]
    assert sha(native) == data["native_source"]["sha256"]
    spec = importlib.util.spec_from_file_location("pingti_git_sources", REPO / "Simulation/PingTi_Arm_v4_new_calib/source_artifacts.py")
    sources = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sources)
    sources.baseline_f3d_bytes()
    current = REPO / "fusion/PingTi_Arm_5DOF_v4.f3d"
    current_sha = sha(current)
    assert current_sha in (sources.BASELINE_F3D_SHA256, "0482ea1c2d5cbedaca712d8cd449715616e53345c698d5dc4238c4eb0ee48ce3", sha(candidate)), "Current F3D has independent changes; do not overwrite"
    if current_sha != sha(candidate):
        backup = REPO / ("fusion/archive/PingTi_Arm_5DOF_v4_before_" + current_sha[:12] + ".f3d")
        backup.parent.mkdir(parents=True, exist_ok=True)
        if not backup.exists():
            shutil.copy2(current, backup)
    urdf = REPO / "Simulation/PingTi_Arm_v4_new_calib/pingti_arm_v4_new_calib.urdf"
    model = json.loads((urdf.parent / "model_manifest.json").read_text())
    assert sha(urdf) == model["urdf_sha256"]
    temporary = current.with_suffix(".f3d.tmp")
    shutil.copy2(candidate, temporary)
    temporary.replace(current)
    manifest = {
        "schema": "pingti-fusion-native-strengthened-assembly/2", "status": "passed",
        "updated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "f3d": str(current.relative_to(REPO)), "f3d_sha256": sha(current), "f3d_bytes": current.stat().st_size,
        "original_commit": sources.BASELINE_COMMIT, "original_f3d_path": sources.BASELINE_F3D_PATH,
        "original_sha256": sources.BASELINE_F3D_SHA256, "backup_policy": "Local only; fusion/archive is Git-ignored. Baseline source is recovered from Git history.",
        "replacement_source": str(native.relative_to(REPO)), "replacement_sha256": sha(native),
        "native_source_version": data["native_source"]["cloud_file"],
        "replacement_component": "elbow_link:1", "replacement_source_body": "Body1",
        "replacement_body": "strong_elbow_link_native_v2", "replacement_native_face_count": 310,
        "geometry_type": "Native analytic BRep copied directly from historical elbow_link Body1; no STL import or mesh conversion.",
        "feature_history": "Assembly uses a copied native base solid. The complete source design with its original 13 timeline entries is retained in replacement_source for feature editing.",
        "excluded_source_body": "Hidden Body3: separate interior solid, excluded from the assembly and print-STL comparison.",
        "reference_stl": "stl_3d_printing/elbow_link.stl", "reference_stl_sha256": data["stl_sha256"],
        "components": 16, "solid_bodies": 18, "joints": 15, "moving_joints": 6,
        "assembly_constraints": "All 15 original connections rebuilt as as-built joints at original placement. Directed datum frames and ordered occurrence pairs retain CAD angle zeros, directions and limits. Base grounded at original placement.",
        "pose_validation_before_export": 72, "pose_validation_after_reopen": 72,
        "validation_report": str(report.relative_to(REPO)), "validation_sha256": sha(report),
        "geometry_match_report": str((OUT/"geometry_match.json").relative_to(REPO)),
        "geometry_match_sha256": sha(OUT/"geometry_match.json"),
        "preview": str((OUT/"fusion_strengthened_assembly.png").relative_to(REPO)),
        "cad_material_note": "Original Fusion materials remain Steel. Native CAD mass is not the SO-ARM-equivalent print/servo mass used by the new_calib URDF.",
        "mass_model": str(urdf.relative_to(REPO)), "urdf_sha256": sha(urdf),
        "urdf_geometry_note": "Strengthened print and simulation STLs replace their original paths. Joint definitions and strengthened mass/inertia remain unchanged. Native BRep volume differs from the STL by 0.027525 percent due to tessellation.",
        "physical_clearance_or_hardware_validation": False
    }
    (REPO/"fusion/assembly_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(json.dumps({k: manifest[k] for k in ("status", "f3d", "f3d_sha256", "f3d_bytes", "replacement_source", "replacement_native_face_count")}, indent=2))

if __name__ == "__main__":
    main()
