"""Package and relocate-load the complete model; requires MuJoCo. No robot I/O."""

import hashlib
import json
import os
from pathlib import Path
import tempfile
import zipfile

import mujoco

OUT = Path(__file__).resolve().parent
ARCHIVE = OUT.with_suffix(".zip")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_archive():
    files = [p for p in sorted(OUT.rglob("*")) if p.is_file() and "__pycache__" not in p.parts
             and "geometry_sources" not in p.parts and p.name not in (".DS_Store", "baseline_491f3da.urdf")]
    checksums = {str(p.relative_to(OUT)): digest(p) for p in files if p.name != "file_checksums.json"}
    (OUT/"file_checksums.json").write_text(json.dumps(checksums, indent=2), encoding="utf-8")
    if OUT/"file_checksums.json" not in files:
        files.append(OUT/"file_checksums.json")
    with zipfile.ZipFile(ARCHIVE, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            archive.write(path, str(Path(OUT.name)/path.relative_to(OUT)))


def main():
    manifest = json.loads((OUT/"model_manifest.json").read_text())
    assert digest(OUT/manifest["urdf"]) == manifest["urdf_sha256"]
    for filename in ("validation.json", "mujoco_validation.json", "pybullet_validation.json"):
        assert json.loads((OUT/filename).read_text())["status"] == "passed"
    (OUT/"package_validation.json").write_text(json.dumps({"status": "pending"}))
    make_archive()
    with tempfile.TemporaryDirectory(prefix="pingti_strong_elbow_") as temporary:
        with zipfile.ZipFile(ARCHIVE) as archive:
            archive.extractall(temporary)
        folder = Path(temporary)/OUT.name
        checksums = json.loads((folder/"file_checksums.json").read_text())
        for relative, expected in checksums.items():
            assert digest(folder/relative) == expected
        previous = os.getcwd()
        try:
            os.chdir(temporary)
            model = mujoco.MjModel.from_xml_path(str(folder/manifest["urdf"]))
            assert model.nq == 6
            assert abs(float(model.body_mass.sum())-manifest["mass_total_kg"]) < 1e-10
        finally:
            os.chdir(previous)
    result = {"status": "passed", "standalone_archive_relocation": True,
              "all_packaged_file_hashes_match": True,
              "urdf_sha256": manifest["urdf_sha256"],
              "mujoco_dof_after_relocation": int(model.nq),
              "mass_kg_after_relocation": float(model.body_mass.sum())}
    (OUT/"package_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    make_archive()
    with zipfile.ZipFile(ARCHIVE) as archive:
        checksums = json.loads(archive.read(OUT.name+"/file_checksums.json"))
        for relative, expected in checksums.items():
            assert hashlib.sha256(archive.read(OUT.name+"/"+relative)).hexdigest() == expected
    print(json.dumps({**result, "archive": str(ARCHIVE), "archive_bytes": ARCHIVE.stat().st_size}, indent=2))


if __name__ == "__main__":
    main()
