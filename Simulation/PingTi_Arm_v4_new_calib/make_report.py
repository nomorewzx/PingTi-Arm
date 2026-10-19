"""Render the saved validation results into a local, self-contained HTML report."""

import hashlib
import html
import json
from pathlib import Path

OUT = Path(__file__).resolve().parent


def load(name):
    return json.loads((OUT/name).read_text())


def table(headers, rows):
    return "<table><thead><tr>"+"".join("<th>"+html.escape(str(v))+"</th>" for v in headers)+"</tr></thead><tbody>"+"".join(
        "<tr>"+"".join("<td>"+html.escape(str(v))+"</td>" for v in row)+"</tr>" for row in rows)+"</tbody></table>"


def main():
    model, cad, mujoco, bullet = [load(name) for name in ("model_manifest.json", "validation.json", "mujoco_validation.json", "pybullet_validation.json")]
    assert all(v["status"] == "passed" for v in (cad,mujoco,bullet))
    ref = model["reference"]
    joints = table(["LeRobot / URDF 关节", "CAD 零位角度", "CAD→URDF 正方向", "URDF 行程"], [
        [j["name"], "%.6f°" % j["cad_angle_at_urdf_zero_deg"], "%+d" % j["cad_to_urdf_sign"], "%.3f° ～ %.3f°" % tuple(j["limits_deg"])]
        for j in model["joints"] if j["type"] != "fixed"])
    masses = table(["连杆", "类别", "估算质量 g", "网格 / CAD 体积差"], [
        [j["name"], "舵机" if j["category"] == "servo" else "打印件", "%.3f" % (j["mass_kg"]*1000), "%.4f%%" % (j["mesh_vs_native_volume_relative_error"]*100)] for j in model["links"]])
    content = """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>PingTi v4 new_calib URDF · 导出验证</title>
<style>
body{font:16px/1.75 system-ui,-apple-system,sans-serif;color:#172b3a;background:#f3f6f8;margin:0}main{max-width:1080px;margin:40px auto;padding:32px;background:white;border-radius:16px}h1{font-size:30px;line-height:1.3}h2{font-size:22px;margin-top:32px}.tag{display:inline-block;background:#e4f4eb;color:#186842;padding:5px 14px;border-radius:20px}p,li{max-width:100ch}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}.card{padding:18px;background:#f0f6f8;border-radius:10px}.card strong{display:block;font-size:25px}table{width:100%%;border-collapse:collapse;font-size:14px}td,th{padding:10px 12px;text-align:left;border-bottom:1px solid #dde5ea}th{background:#edf3f6}code,pre{font-family:ui-monospace,monospace;font-size:13px}pre{white-space:pre-wrap;overflow-wrap:anywhere;padding:16px;background:#102432;color:#f4f8fb;border-radius:8px}a{color:#006b9a}.note{padding:16px 20px;background:#fff7e4;border-left:4px solid #d19a25}.gallery{display:grid;grid-template-columns:1fr 1fr;gap:16px}figure{margin:0}img{width:100%%;border:1px solid #e5e9ec;border-radius:8px}figcaption{font-size:14px;color:#4b6070}.hash{overflow-wrap:anywhere;font:12px/1.6 ui-monospace,monospace}@media(max-width:760px){main{margin:0;padding:18px;border-radius:0}.cards,.gallery{grid-template-columns:1fr}table{font-size:12px}td,th{padding:7px}}
</style><main>
<span class="tag">CAD 验证、MuJoCo、PyBullet：通过</span>
<h1>PingTi v4 new_calib URDF</h1>
<p>在本地 Fusion 360 中，重新导入 <code>PingTi_Arm_5DOF_v4.f3d</code> 到独立临时文档，读取原生组件、关节轴、限位及物性，导出 16 个以米为单位的网格。随后重新建立零位坐标变换，并使用 Fusion 求解器产生的姿态做独立对照。原始 F3D、v3 URDF 均未改动。</p>
<p><a href="pingti_arm_v4_new_calib.urdf">打开新 URDF</a> · <a href="model_manifest.json">建模来源与参数</a> · <a href="validation.json">CAD 验证明细</a> · <a href="mujoco_validation.json">MuJoCo 结果</a> · <a href="pybullet_validation.json">PyBullet 结果</a></p>
<div class="cards"><div class="card"><strong>72 组</strong>Fusion 独立姿态</div><div class="card"><strong>5 + 1</strong>机械臂关节 + 夹爪关节</div><div class="card"><strong>%(mass).3f kg</strong>SO 参考假设下的总质量</div></div>
<div class="gallery"><figure><img src="zero_pose.png" alt="PingTi 新 URDF 零位，夹爪闭合"><figcaption>五个机械臂关节均为零；夹爪命令 0，模型夹爪角度 0。</figcaption></figure><figure><img src="gripper_open.png" alt="PingTi 新 URDF，夹爪打开"><figcaption>机械臂仍处于零位；夹爪命令 100，模型夹爪角度 102°。</figcaption></figure></div>
<h2>零位与方向</h2>
<p>底座、肩、肘、腕俯仰采用 CAD 名义行程中点。腕滚转沿用 SO ARM 参考姿态：以本地 <code>so101_new_calib.urdf</code> 为具体参考，匹配腕俯仰正轴到夹爪铰链正轴、绕腕滚转正轴的有向夹角 <strong>%(wrist).6f°</strong>。这会得到 PingTi 自身 CAD 中约 92.789130° 的腕角，不能把它当作真机额外补偿值。</p>
%(joints)s
<p>表中的 CAD 角度和方向只用于从 CAD 构建 URDF。LeRobot 的五个机械臂角度输入本模型时只需度→弧度，不应再次应用表中的符号或偏移。正方向采用 SO new_calib 的名义约定：三个俯仰关节正轴平行，腕滚转正轴朝向前臂。</p>
<p>Fusion 对此文档中驱动关节的数值舍入至 0.1°。腕部参考姿态从实际导出的 92.8° 沿已知转轴、枢轴解析修正 %(residual).9f°，以保留 SO 参考角；没有对验证姿态逐个拟合。</p>
<h2>质量、质心与惯性</h2>
<p>采用用户指定的共同材料／有效密度与舵机同质量假设。SO 参考文件没有独立给出打印密度和舵机质量，因此选择两个可分离的组件推算，而非复制整条 SO 连杆的质量或惯性：</p>
<ol><li>SO 无舵机活动夹爪质量 12 g ÷ 其网格体积，得到有效打印密度 <strong>%(density).3f kg/m³</strong>。</li><li>SO 固定夹爪连杆总质量 87 g，减去同一密度下打印件质量，得到单颗舵机参考质量 <strong>%(servo).3f g</strong>。</li><li>PingTi 打印件按自己的 CAD 实体体积计算质量；8 颗舵机均采用上述质量，包含两颗 STS3250 的明确近似。</li><li>以每个 PingTi 原生组件的质心与几何惯性，按新质量缩放并变换到新连杆坐标。全部实体惯性均为正定，满足惯性三角不等式。</li></ol>
<p>这是有效密度近似，不是实心耗材密度，也不等于已知填充率。质心及惯性按均匀等效密度建模，未识别打印壳层、内部填充或舵机内部零件的实际质量分布。SO 参考 commit：<code>%(so_commit)s</code>；引用文件、哈希和计算公式保存在 <a href="model_manifest.json">manifest</a>，参考副本附带原许可证。</p>
%(masses)s
<h2>验证结果与可解释范围</h2>
<ul><li>原生 CAD 关节图：16 个实体组件、15 个连接；新模型加一个质量为 0 的输出端参考 frame。</li><li>72 组 Fusion 姿态包含机械端点、逐关节扰动、48 组随机组合和腕部参考姿态。逐组件核对完整刚体变换；自写 FK 最大参考点误差 %(fk).3g mm，属于数值一致性结果。</li><li>MuJoCo %(mj_version)s 独立导入与 72 组 FK 对照通过；系统质量矩阵在全部样本中正定，最小特征值 %(eig).6g。</li><li>PyBullet 独立导入与 72 组 FK 对照通过；最大参考点误差 %(bullet_error).6f mm。单精度返回值造成的差异不代表实物精度。</li><li>STL 网格与 CAD 的最大体积相对差 %(meshvol).4f%%，最大质心差 %(meshcom).4f mm；CPU 渲染已检查零位、侧视和夹爪开合。</li><li>原始 F3D 与原 URDF 的 SHA-256 均与导出前一致。</li></ul>
<div class="note"><strong>这些结果确认模型与 CAD 的一致性，不是毫米级真机精度证明。</strong> 尚未实测电机正方向、校准端点、腕部参考摆姿、夹爪开合和实际 TCP。CAD 端点样本仅用于 FK 验证，不代表碰撞可行姿态。网格在物理引擎中可能被凸化；未验证接触任务、抓取、控制器增益或真实负载。</div>
<h2>在 LeRobot / 仿真中使用</h2>
<p>URDF 与 <code>meshes/</code> 保持相对路径，可以整目录搬运。PingTi 配置需使用 <code>use_degrees=True</code>。五个机械臂关节按度转弧度；夹爪 0～100 映射到 0～102°。转换函数位于 <a href="joint_mapping.py">joint_mapping.py</a>，不会连接或控制机械臂。</p>
<pre>from joint_mapping import lerobot_to_urdf
q_rad = lerobot_to_urdf(robot_observation)
# q_rad keys: shoulder_pan, shoulder_lift, elbow_flex,
#             wrist_flex, wrist_roll, gripper</pre>
<p>肩、肘两组镜像舵机各对应一个机械关节，URDF 不增加独立 secondary 自由度。<code>tool_mount_link</code> 位于腕滚转输出轴原点，可作为后续附加实测 TCP 的基准；它不是指尖抓取中心。</p>
<p>位置规划参数暂设 velocity=1 rad/s；单舵机 effort=3.35 Nm，双舵机关节 6.70 Nm，阻尼与摩擦参考 SO 模型。这些是近似模型参数，既不是实测 PingTi 性能，也不由 URDF 自动约束真实驱动器。</p>
<h2>复现</h2>
<p>Fusion 导出脚本位于仓库 <code>Simulation/validation/urdf_audit_20261019/FusionNewCalib/</code>。其导出来源与完整姿态保存在 <a href="provenance/fusion_export.json">fusion_export.json</a>。在安装相应 Python 包的环境中运行：</p>
<pre>python build_model.py       # NumPy + SciPy；读取本地 SO-ARM100 参考模型
python verify_mujoco.py     # MuJoCo + NumPy
python verify_pybullet.py   # PyBullet + NumPy；同时输出渲染图
python make_report.py</pre>
<h2>文件校验</h2><p>新 URDF</p><p class="hash">%(new_sha)s</p><p>保留的 v3 URDF</p><p class="hash">%(old_sha)s</p><p>源 F3D</p><p class="hash">%(cad_sha)s</p>
<p>几何来源：<a href="https://github.com/nomorewzx/PingTi-Arm">PingTi-Arm</a>；零位与质量参考：<a href="https://github.com/TheRobotStudio/SO-ARM100">TheRobotStudio/SO-ARM100</a>。本报告与导出文件生成于 2026-10-19。</p>
</main></html>""" % {
        "mass": model["mass_total_kg"], "wrist": ref["wrist_pitch_to_jaw_signed_angle_deg"],
        "joints": joints, "masses": masses, "density": ref["effective_print_density_kg_m3"],
        "servo": ref["servo_mass_kg"]*1000, "so_commit": ref["local_commit"],
        "residual": model["fusion_reference_rounding"]["analytic_residual_rotation_deg"],
        "fk": cad["max_fk_point_error_mm"], "mj_version": mujoco["version"],
        "eig": mujoco["minimum_generalized_mass_matrix_eigenvalue"],
        "bullet_error": bullet["max_rigid_reference_point_error_mm"],
        "meshvol": cad["max_native_mesh_volume_relative_error"]*100,
        "meshcom": cad["max_native_mesh_com_error_mm"], "new_sha": model["urdf_sha256"],
        "old_sha": model["original_urdf_sha256"], "cad_sha": model["source_f3d_sha256"]}
    (OUT/"validation_report.html").write_text(content, encoding="utf-8")
    print(OUT/"validation_report.html")


if __name__ == "__main__":
    main()
