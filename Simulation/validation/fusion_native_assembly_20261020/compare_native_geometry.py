"""Match historical native body to the selected print STL and end mounts."""
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
from rtree import index
from scipy.spatial import cKDTree

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[2]


def triangles(path, scale=1.):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.ndarray(count, dtype=dtype, buffer=raw, offset=84)["v"].astype(float) * scale


def distances_to_triangles(point, t):
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    ab, ac, ap = b-a, c-a, point-a
    normal = np.cross(ab, ac)
    norm2 = np.einsum("ij,ij->i", normal, normal)
    plane = np.einsum("ij,ij->i", ap, normal)
    projected = ap - normal * np.divide(plane, norm2, out=np.zeros_like(plane), where=norm2 > 1e-30)[:, None]
    aa = np.einsum("ij,ij->i", ab, ab)
    cc = np.einsum("ij,ij->i", ac, ac)
    cross = np.einsum("ij,ij->i", ab, ac)
    pa = np.einsum("ij,ij->i", projected, ab)
    pc = np.einsum("ij,ij->i", projected, ac)
    det = aa*cc-cross*cross
    u = np.divide(pa*cc-pc*cross, det, out=np.full_like(det, -1.), where=det > 1e-30)
    v = np.divide(pc*aa-pa*cross, det, out=np.full_like(det, -1.), where=det > 1e-30)
    inside = (u >= 0) & (v >= 0) & (u+v <= 1) & (norm2 > 1e-30)
    dist = np.where(inside, np.divide(plane*plane, norm2, out=np.full_like(plane, np.inf), where=norm2 > 1e-30), np.inf)
    for p, q in ((a, b), (b, c), (c, a)):
        edge = q-p
        denominator = np.einsum("ij,ij->i", edge, edge)
        f = np.divide(np.einsum("ij,ij->i", point-p, edge), denominator, out=np.zeros_like(denominator), where=denominator > 1e-30)
        residual = point - (p + np.clip(f, 0., 1.)[:, None] * edge)
        dist = np.minimum(dist, np.einsum("ij,ij->i", residual, residual))
    return np.sqrt(dist.min())


def sample(t):
    return np.unique(np.concatenate((t.reshape(-1, 3), t.mean(axis=1), (t[:, 0]+t[:, 1])/2,
                                    (t[:, 1]+t[:, 2])/2, (t[:, 2]+t[:, 0])/2)), axis=0)


def directed(points, target):
    bounds = np.concatenate((target.min(axis=1), target.max(axis=1)), axis=1)
    prop = index.Property()
    prop.dimension = 3
    tree = index.Index(((i, tuple(box), None) for i, box in enumerate(bounds)), properties=prop)
    upper, _ = cKDTree(target.reshape(-1, 3)).query(points)
    distances = []
    for p, radius in zip(points, upper):
        radius += 1e-10
        ids = list(tree.intersection(tuple(np.concatenate((p-radius, p+radius)))))
        assert ids
        distances.append(distances_to_triangles(p, target[ids]))
    d = np.array(distances) * 1000.
    return {"sample_count": len(d), "max_mm": float(d.max()), "p99_mm": float(np.quantile(d, .99)), "rms_mm": float(np.sqrt((d*d).mean())),
            "worst_samples": [{"point_m": points[i].tolist(), "distance_mm": float(d[i])} for i in np.argsort(d)[-10:][::-1]]}


def main():
    probe = json.loads((OUT / "native_body_probe.json").read_text())
    body, = [b for b in probe["bodies"] if b["name"] == "Body1"]
    selected = REPO / "stl_3d_printing/elbow_link.stl"
    stl = triangles(selected, .001)
    native = triangles(OUT / body["audit_mesh"])
    # Fixed interface observations from the accepted original-CAD audit are
    # retained in the report, independently of the candidate native surfaces.
    reference = json.loads((OUT / "geometry_match.json").read_text())
    old = [item["old_cylinder"] for item in reference["end_interface_cylinder_matches"]]
    cylinders = [s for s in body["surfaces"] if s["type"] == "adsk::core::Cylinder"]
    # The end interfaces are outside the changed central reinforcement region.
    end_cylinders = [s for s in old if s["type"] == "adsk::core::Cylinder" and (s["origin_m"][0] < .14 or s["origin_m"][0] > .30)]
    matched = []
    for source in end_cylinders:
        matches = [i for i, target in enumerate(cylinders)
                   if abs(source["radius_m"]-target["radius_m"]) < 1e-7
                   and abs(abs(np.dot(source["axis"], target["axis"]))-1.) < 1e-7
                   and np.linalg.norm(np.cross(np.subtract(source["origin_m"], target["origin_m"]), source["axis"])) < 1e-7]
        assert matches, source
        matched.append({"old_cylinder": source, "matching_native_cylinder_indices": matches})
    result = {"status": "started", "historical_design": "shoulder_to_elbow_link", "version": 2,
              "component": "elbow_link", "body": "Body1", "native_faces": body["faces"],
              "surface_types": body["surface_types"], "native_volume_cm3": body["properties"]["volume_cm3"],
              "selected_stl_sha256": hashlib.sha256(selected.read_bytes()).hexdigest(),
              "excluded_hidden_body": "Body3 (separate interior solid, not part of the selected print STL)",
              "end_interface_cylinders_checked": len(matched), "end_interface_cylinder_matches": matched,
              "bbox_max_coordinate_difference_mm": float(np.max(np.abs(np.array(body["bbox_m"])-np.stack((stl.min(axis=(0, 1)), stl.max(axis=(0, 1))))))*1000),
              "comparison_method": "Both directions: all tessellation vertices, triangle centroids and edge midpoints; exact point-to-triangle distances with R-tree bounding-box candidates. Sampled surface comparison, not an analytic Hausdorff proof."}
    result["stl_to_native_tessellation"] = directed(sample(stl), native)
    result["native_tessellation_to_stl"] = directed(sample(native), stl)
    result["selected_stl_volume_cm3"] = float(np.einsum("ij,ij->i", stl[:, 0], np.cross(stl[:, 1], stl[:, 2])).sum()/6*1e6)
    result["native_volume_difference_percent"] = (result["native_volume_cm3"]/result["selected_stl_volume_cm3"]-1)*100.
    (OUT / "geometry_match.json").write_text(json.dumps(result, indent=2))
    assert result["bbox_max_coordinate_difference_mm"] < .001
    assert result["stl_to_native_tessellation"]["max_mm"] < .1
    assert result["native_tessellation_to_stl"]["max_mm"] < .1
    assert abs(result["native_volume_difference_percent"]) < .1
    result["status"] = "passed"
    (OUT / "geometry_match.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({"status": result["status"], "native_faces": body["faces"], "stl_to_native_max_mm": result["stl_to_native_tessellation"]["max_mm"], "native_to_stl_max_mm": result["native_tessellation_to_stl"]["max_mm"], "end_interface_cylinders_checked": len(matched)}, indent=2))


if __name__ == "__main__":
    main()
