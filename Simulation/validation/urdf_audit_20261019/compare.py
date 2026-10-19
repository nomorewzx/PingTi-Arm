"""Compare the existing URDF with measurements exported by the local Fusion app.

Run from any directory: python3 /absolute/path/to/compare.py
Requires NumPy and SciPy. Only writes audit results next to this script.
"""

import hashlib
import html
import json
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


OUT = Path(__file__).resolve().parent
REPO = OUT.parents[2]
URDF = REPO / "Simulation/PingTi_Arm_v3/urdf/PingTi_Arm_v3.urdf"
CAD = REPO / "fusion/PingTi_Arm_5DOF_v4.f3d"


def vector(text):
    return np.fromstring(text, sep=" ")


def unit(v):
    return v / np.linalg.norm(v)


def matrix(value):
    result = np.array(value["row_major_cm"], dtype=float).reshape(4, 4)
    result[:3, 3] *= 0.01
    return result


def point(transform, value):
    return transform[:3, :3] @ value + transform[:3, 3]


def signed_angle(a, b, axis):
    return float(np.arctan2(unit(axis) @ np.cross(unit(a), unit(b)), unit(a) @ unit(b)))


def stl_vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {path}")
    rows = np.frombuffer(data, dtype=np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")
    ]), offset=84, count=count)
    return rows["vertices"].astype(float).reshape(-1, 3)


def analyze():
    snapshot = json.loads((OUT / "fusion_snapshot.json").read_text())
    poses = json.loads((OUT / "fusion_pose_samples.json").read_text())
    root = ET.parse(URDF).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = list(root.findall("joint"))
    moving = {j.attrib["name"]: j for j in joints if j.attrib["type"] != "fixed"}
    cad_joints = {j["name"]: j for j in snapshot["joints"] if j["motion_type"] == 1}
    occurrences = {re.sub(r"[ :()]", "_", o["name"]): o for o in snapshot["occurrences"]}
    occurrences["base_link"] = next(o for o in snapshot["occurrences"]
                                      if o["component_name"].startswith("base_link"))
    occurrences = {name: occurrences[name] for name in links}
    offsets = {}

    def fk(cad_angles):
        transforms = {"base_link": np.eye(4)}
        joint_frames = {}
        pending = joints[:]
        while pending:
            progress = False
            for j in pending[:]:
                parent = j.find("parent").attrib["link"]
                child = j.find("child").attrib["link"]
                if parent not in transforms:
                    continue
                origin = j.find("origin")
                relative = np.eye(4)
                relative[:3, 3] = vector(origin.attrib["xyz"])
                relative[:3, :3] = Rotation.from_euler(
                    "xyz", vector(origin.attrib.get("rpy", "0 0 0"))).as_matrix()
                joint_frames[j.attrib["name"]] = transforms[parent] @ relative
                if j.attrib["name"] in moving:
                    name = j.attrib["name"]
                    q = cad_angles[name]
                    q = np.arctan2(np.sin(q), np.cos(q)) + offsets.get(name, 0.0)
                    rotation = np.eye(4)
                    rotation[:3, :3] = Rotation.from_rotvec(
                        unit(vector(j.find("axis").attrib["xyz"])) * q).as_matrix()
                    relative = relative @ rotation
                transforms[child] = transforms[parent] @ relative
                pending.remove(j)
                progress = True
            if not progress:
                raise ValueError("Disconnected or cyclic URDF joint graph")
        return transforms, joint_frames

    angles = poses["baseline"]["rotations_rad"]
    # Infer only the two explicit coordinate-convention offsets from axis vectors.
    # No optimization against sample poses or per-pose fitting is performed.
    _, unaligned_frames = fk(angles)
    shoulder_axis = unaligned_frames["shoulder_pitch"][:3, :3] @ unit(
        vector(moving["shoulder_pitch"].find("axis").attrib["xyz"]))
    yaw_axis = unit(vector(moving["base_yaw"].find("axis").attrib["xyz"]))
    offsets["base_yaw"] = signed_angle(shoulder_axis, np.array(cad_joints["shoulder_pitch"]["axis_vector"]), yaw_axis)
    _, yaw_aligned_frames = fk(angles)
    gripper_axis = yaw_aligned_frames["gripper_moving"][:3, :3] @ unit(
        vector(moving["gripper_moving"].find("axis").attrib["xyz"]))
    roll_axis = yaw_aligned_frames["wrist_roll"][:3, :3] @ unit(
        vector(moving["wrist_roll"].find("axis").attrib["xyz"]))
    offsets["wrist_roll"] = signed_angle(gripper_axis, np.array(cad_joints["gripper_moving"]["axis_vector"]), roll_axis)
    baseline, frames = fk(angles)
    base_com = vector(links["base_link"].find("inertial/origin").attrib["xyz"])
    root_transform = np.eye(4)
    root_transform[:3, 3] = np.array(occurrences["base_link"]["physical_properties"]["center_of_mass_m"]) - base_com

    joint_rows = []
    for name, j in moving.items():
        f = root_transform @ frames[name]
        axis = unit(f[:3, :3] @ unit(vector(j.find("axis").attrib["xyz"])))
        cj = cad_joints[name]
        delta = np.array(cj["geometry_two_transform"]["translation_m"]) - f[:3, 3]
        cad_axis = unit(np.array(cj["axis_vector"]))
        # Point origins may differ along the same shaft. Perpendicular line error matters.
        line_error = np.linalg.norm(np.cross(delta, axis)) * 1000
        limits = j.find("limit")
        joint_rows.append({
            "joint": name,
            "axis_angle_error_deg": float(np.degrees(np.arctan2(np.linalg.norm(np.cross(axis, cad_axis)), axis @ cad_axis))),
            "axis_line_offset_mm": float(line_error),
            "origin_delta_mm": (delta * 1000).tolist(),
            "urdf_type": j.attrib["type"],
            "urdf_limits_deg": [float(np.degrees(float(limits.attrib[k]))) for k in ("lower", "upper")] if limits is not None else None,
            "cad_limits_deg": [float(np.degrees(cj["rotation_limits"][k])) for k in ("minimumValue", "maximumValue")]
                if cj["rotation_limits"]["isMinimumValueEnabled"] else None,
            "urdf_effort_Nm": float(limits.attrib["effort"]) if limits is not None else None,
            "urdf_velocity_rad_s": float(limits.attrib["velocity"]) if limits is not None else None,
        })

    body_rows = []
    for name, link in links.items():
        cad = occurrences[name]
        prop = cad["physical_properties"]
        urdf_mass = float(link.find("inertial/mass").attrib["value"])
        urdf_com = vector(link.find("inertial/origin").attrib["xyz"])
        predicted_com = point(root_transform @ baseline[name], urdf_com)
        inertia = link.find("inertial/inertia").attrib
        moment = np.array([[float(inertia["i" + a + b]) if a <= b else float(inertia["i" + b + a])
                            for b in "xyz"] for a in "xyz"])
        eigenvalues = np.linalg.eigvalsh(moment)
        mesh = link.find("visual/geometry/mesh")
        vertices = stl_vertices(URDF.parent / mesh.attrib["filename"])
        vertices *= vector(mesh.attrib.get("scale", "1 1 1"))
        origin = link.find("visual/origin")
        rotation = Rotation.from_euler("xyz", vector(origin.attrib.get("rpy", "0 0 0"))).as_matrix()
        vertices = vertices @ rotation.T + vector(origin.attrib["xyz"])
        tr = root_transform @ baseline[name]
        vertices = vertices @ tr[:3, :3].T + tr[:3, 3]
        bbox = np.stack((vertices.min(axis=0), vertices.max(axis=0)))
        body_rows.append({
            "link": name, "cad_occurrence": cad["full_path"],
            "urdf_mass_kg": urdf_mass, "cad_mass_kg": prop["mass_kg"],
            "mass_difference_g": (prop["mass_kg"] - urdf_mass) * 1000,
            "cad_materials": sorted({b["material_name"] for b in cad["bodies"]}),
            "com_offset_at_aligned_saved_pose_mm": float(np.linalg.norm(predicted_com - prop["center_of_mass_m"]) * 1000),
            "visual_bbox_corner_max_difference_mm": float(np.max(np.abs(bbox - np.array(cad["bbox_m"]))) * 1000),
            "inertia_eigenvalues_kg_m2": eigenvalues.tolist(),
            "inertia_positive_definite": bool(eigenvalues.min() > 0),
            "inertia_triangle_inequality": bool(eigenvalues[-1] <= sum(eigenvalues[:2]) + 1e-9),
        })

    motion_rows = []
    cad_baseline = {name: matrix(value) for name, value in poses["baseline"]["occurrences"].items()}
    for index, sample in enumerate(poses["samples"]):
        if "error" in sample:
            motion_rows.append({"index": index, "error": sample["error"]})
            continue
        current = sample["pose"]
        transforms, _ = fk(current["rotations_rad"])
        errors = []
        for name, cad in occurrences.items():
            delta_urdf = root_transform @ transforms[name] @ np.linalg.inv(baseline[name]) @ np.linalg.inv(root_transform)
            delta_cad = matrix(current["occurrences"][cad["full_path"]]) @ np.linalg.inv(cad_baseline[cad["full_path"]])
            # Four common baseline reference points per component isolate motion from shape changes.
            center = np.array(cad["physical_properties"]["center_of_mass_m"])
            reference_points = [center] + [center + np.eye(3)[i] * 0.02 for i in range(3)]
            point_errors = [float(np.linalg.norm(point(delta_urdf, p) - point(delta_cad, p)) * 1000)
                            for p in reference_points]
            angle = float(np.degrees(Rotation.from_matrix(delta_urdf[:3, :3] @ delta_cad[:3, :3].T).magnitude()))
            errors.append({"link": name, "max_reference_point_error_mm": max(point_errors),
                           "orientation_error_deg": angle})
        motion_rows.append({
            "index": index, "kind": sample["driven_joint"],
            "requested_delta_deg": sample.get("requested_delta_deg"),
            "actual_cad_angles_deg": {name: float(np.degrees(np.arctan2(np.sin(q), np.cos(q))))
                                      for name, q in current["rotations_rad"].items()},
            "max_reference_point_error_mm": max(x["max_reference_point_error_mm"] for x in errors),
            "max_orientation_error_deg": max(x["orientation_error_deg"] for x in errors),
            "per_link": errors,
        })

    restoration_error = max(float(np.max(np.abs(matrix(poses["restored"]["occurrences"][name]) - value)))
                            for name, value in cad_baseline.items())
    result = {
        "schema": "pingti-urdf-fusion-comparison/1",
        "scope": "CAD kinematic consistency only; not hardware, collision, payload, or dynamics certification",
        "fusion_version": snapshot["fusion_version"],
        "f3d_path": str(CAD), "f3d_sha256": hashlib.sha256(CAD.read_bytes()).hexdigest(),
        "f3d_hash_matches_export": hashlib.sha256(CAD.read_bytes()).hexdigest() == snapshot["source_sha256"],
        "urdf_path": str(URDF), "urdf_sha256": hashlib.sha256(URDF.read_bytes()).hexdigest(),
        "counts": {"links": len(links), "joints": len(joints), "moving_joints_including_gripper": len(moving),
                   "motion_samples": len(motion_rows), "reference_points_per_component": 4},
        "cad_from_urdf_root_transform_m": root_transform.tolist(),
        "cad_to_urdf_joint_offsets_deg": {name: float(np.degrees(q)) for name, q in offsets.items()},
        "offset_warning": "Inferred CAD coordinate convention only. Do not apply these directly to motor encoders.",
        "comparison_method": "Compare rigid displacement from the saved pose on common reference points; two fixed offsets inferred from saved joint axes, no per-pose fitting.",
        "maximum_motion_reference_point_error_mm": max(row.get("max_reference_point_error_mm", 0) for row in motion_rows),
        "maximum_motion_orientation_error_deg": max(row.get("max_orientation_error_deg", 0) for row in motion_rows),
        "restore_errors": poses["restore_errors"], "restored_transform_max_absolute_difference": restoration_error,
        "total_urdf_mass_kg": sum(row["urdf_mass_kg"] for row in body_rows),
        "total_cad_mass_kg": sum(row["cad_mass_kg"] for row in body_rows),
        "tcp_frame_present": any(re.search(r"tcp|tool0|tool_tip", name, re.I) for name in links),
        "joint_comparison": joint_rows, "body_comparison": body_rows, "motion_comparison": motion_rows,
    }
    (OUT / "comparison.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def report(data):
    def table(headers, rows):
        return "<table><thead><tr>" + "".join(f"<th>{html.escape(str(x))}</th>" for x in headers) + "</tr></thead><tbody>" + "".join(
            "<tr>" + "".join(f"<td>{html.escape(str(x))}</td>" for x in row) + "</tr>" for row in rows) + "</tbody></table>"

    joint_table = table(["关节", "轴线偏差 mm", "轴方向差 °", "URDF 限位 °", "CAD 限位 °"], [
        [j["joint"], f'{j["axis_line_offset_mm"]:.4f}', f'{j["axis_angle_error_deg"]:.6f}',
         str([round(x, 2) for x in j["urdf_limits_deg"]]) if j["urdf_limits_deg"] else "continuous",
         str([round(x, 2) for x in j["cad_limits_deg"]]) if j["cad_limits_deg"] else "CAD 未设置"]
        for j in data["joint_comparison"]])
    body_table = table(["组件", "URDF 质量 kg", "CAD 质量 kg", "质量差 g", "已对齐姿态的质心差 mm", "CAD 材料"], [
        [b["link"], f'{b["urdf_mass_kg"]:.6f}', f'{b["cad_mass_kg"]:.6f}', f'{b["mass_difference_g"]:.3f}',
         f'{b["com_offset_at_aligned_saved_pose_mm"]:.3f}', ", ".join(b["cad_materials"])] for b in data["body_comparison"]])
    motion_table = table(["序号", "检查", "最大参考点运动差 mm", "最大方向差 °"], [
        [r["index"] + 1, r.get("kind", "error") + (f' {r["requested_delta_deg"]:+g}°' if r.get("requested_delta_deg") else ""),
         f'{r.get("max_reference_point_error_mm", 0):.5f}', f'{r.get("max_orientation_error_deg", 0):.8f}'] for r in data["motion_comparison"]])
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>PingTi URDF 与 Fusion 核验</title>
<style>body{{max-width:1160px;margin:48px auto;padding:0 24px;font:16px/1.7 system-ui;color:#172b3a;background:#f8fafc}}h1,h2{{line-height:1.35}}h2{{margin-top:36px}}table{{border-collapse:collapse;width:100%;background:white;font-size:14px;margin:20px 0}}td,th{{text-align:left;border:1px solid #dbe3e9;padding:9px}}th{{background:#edf3f7}}code{{background:#e8eef3;padding:2px 5px}}.note{{background:#fff3d7;padding:20px;border-left:5px solid #c48a16}}.evidence{{background:#e7f4ef;padding:20px}}a{{color:#12608a}}</style>
<h1>PingTi：现有 URDF 与本机 Fusion 装配核验</h1><p>2026-10-19 · Fusion {html.escape(data["fusion_version"])} · CAD v4 / URDF v3</p>
<p class="note"><strong>结论：运动链在本次 CAD 对照中基本一致；当前文件仍需要零位、限位、实物质量与 TCP 标定，不能直接视为真机已验收模型。</strong></p>
<p class="evidence">已在 Fusion 打开原始 F3D，读取 16 个组件与 15 个关节，执行 {data["counts"]["motion_samples"]} 组姿态检查：6 个活动关节各 ±5°、一组全关节 +5°、20 组固定随机种子的组合姿态。最大共同参考点运动差 <strong>{data["maximum_motion_reference_point_error_mm"]:.4f} mm</strong>，最大方向差 {data["maximum_motion_orientation_error_deg"]:.8f}°。每个组件使用质心及沿三轴偏移 20 mm 的三个点。</p>
<h2>怎么理解这个结果</h2><p>比较的是从相同基准姿态出发的刚体运动，避免把零位不同或新版实体形状变化误判为关节方向错误。只有两个固定坐标约定偏移：base_yaw={data["cad_to_urdf_joint_offsets_deg"]["base_yaw"]:.6f}°，wrist_roll={data["cad_to_urdf_joint_offsets_deg"]["wrist_roll"]:.6f}°。它们由基准姿态的关节轴推导，未针对测试姿态拟合，也不是实物编码器校准值。</p>
<p>组合姿态范围：base_yaw、shoulder_pitch、wrist_pitch 各 [-70°,70°]；elbow_pitch [-70°,80°]；wrist_roll [-90°,90°]；gripper_moving [-5°,60°]。这是有限样本的 CAD 运动学检查，不覆盖全部工作空间，不证明碰撞安全、抓取精度、负载或动力学正确。</p>
<h2>关节对比</h2><p>轴线上原点可以沿转轴选择不同位置，因此表格比较垂直于轴的偏差。肩关节 CAD 的负向限位为 -110°，URDF 为 -100°。腕部 continuous 源于 CAD 没有设置限位，不意味着真机可无限旋转。</p>{joint_table}
<h2>组件和质量属性</h2><p>CAD 的全部实体材料为 Steel。URDF 总质量 {data["total_urdf_mass_kg"]:.6f} kg，CAD 总质量 {data["total_cad_mass_kg"]:.6f} kg；这些是钢密度 CAD 数值，不能代表打印件、舵机和附件的实物质量。肩连杆与肘连杆质量/几何也存在版本差异。</p>{body_table}
<h2>仍需要做的真机验收</h2><ol><li>逐关节确认电机零点、正方向、角度单位、双电机联动和实物限位，记录 LeRobot 到 URDF 的映射。</li><li>定义夹爪 TCP 和开合宽度映射；现有 URDF 没有单独 TCP frame。</li><li>用实测长度、逐件称重和真实材料更新模型；舵机质量用实测或厂商数据，打印件考虑填充率。</li><li>以多个实物姿态测量 TCP 位置，评估未参与校准姿态的误差；精度门槛按夹爪与目标间隙制定。</li><li>另行验证碰撞几何、速度/加速度限值、负载和接触行为。导出器固定填写的 effort=100、velocity=100 不能作为真机限制。</li></ol>
<h2>可追溯性</h2><p>原始 F3D 和 URDF 没有被本轮写入。CAD 临时驱动后已恢复，恢复错误数 {len(data["restore_errors"])}，组件矩阵最大绝对差 {data["restored_transform_max_absolute_difference"]:.3g}。F3D 哈希与导出时一致：{data["f3d_hash_matches_export"]}。</p>
<p>F3D SHA256：<code>{data["f3d_sha256"]}</code><br>URDF SHA256：<code>{data["urdf_sha256"]}</code></p>
<p><a href="fusion_snapshot.json">Fusion 原始元数据</a> · <a href="fusion_pose_samples.json">Fusion 姿态原始记录</a> · <a href="comparison.json">完整比较数据</a> · <a href="FusionAudit/FusionAudit.py">Fusion 采集脚本</a> · <a href="compare.py">离线比较脚本</a></p>
<h2>33 组运动检查记录</h2>{motion_table}</html>'''
    (OUT / "report.html").write_text(page, encoding="utf-8")


if __name__ == "__main__":
    result = analyze()
    report(result)
    print(json.dumps({key: result[key] for key in (
        "counts", "cad_to_urdf_joint_offsets_deg", "maximum_motion_reference_point_error_mm",
        "maximum_motion_orientation_error_deg", "restored_transform_max_absolute_difference",
        "total_urdf_mass_kg", "total_cad_mass_kg", "f3d_hash_matches_export"
    )}, indent=2))
