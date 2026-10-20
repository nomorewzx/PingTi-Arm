"""Recover baseline inputs from Git history without committing backup copies."""
import hashlib
from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[2]
BASELINE_COMMIT = "491f3da"
BASELINE_F3D_PATH = "fusion/PingTi_Arm_5DOF_v4.f3d"
BASELINE_F3D_SHA256 = "d593c01c38da6e048790d1c9c18e48add82de338029c5f8759322417adfaf582"


def git_blob(path, expected_sha256, commit=BASELINE_COMMIT):
    result = subprocess.run(["git", "-C", str(REPO), "show", commit + ":" + path], check=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert hashlib.sha256(result.stdout).hexdigest() == expected_sha256, "Baseline Git object changed: " + path
    return result.stdout


def baseline_f3d_bytes():
    return git_blob(BASELINE_F3D_PATH, BASELINE_F3D_SHA256)


def ensure_baseline_f3d():
    """Fusion needs a file path; materialize a verified, Git-ignored local cache."""
    cache = REPO / "fusion/archive/PingTi_Arm_5DOF_v4_original.f3d"
    if not cache.exists():
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_bytes(baseline_f3d_bytes())
    assert hashlib.sha256(cache.read_bytes()).hexdigest() == BASELINE_F3D_SHA256
    return cache


def baseline_urdf_bytes(config):
    return git_blob(config["baseline_urdf_path"], config["baseline_urdf_sha256"], config["baseline_commit"])
