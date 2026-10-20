"""Inspect persisted native solids, not tessellated reconstructions."""
import collections
import importlib.util
import json
from pathlib import Path
import traceback
import adsk.core
import adsk.fusion

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[2]

def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

def run(context):
    app = adsk.core.Application.get()
    previous = app.activeDocument
    doc = None
    result = {"status": "started", "bodies": []}
    try:
        audit = module("native_probe_audit", REPO / "Simulation/validation/urdf_audit_20261019/FusionAudit/FusionAudit.py")
        exporter = module("native_probe_export", REPO / "Simulation/validation/urdf_audit_20261019/FusionNewCalib/FusionNewCalib.py")
        exporter.OUT = OUT
        doc = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(REPO / "fusion/parts/shoulder_to_elbow_link_v2_native_source.f3d")))
        design = adsk.fusion.Design.cast(app.activeProduct)
        component, = [o.component for o in design.rootComponent.allOccurrences if o.component.name == "elbow_link"]
        for i, body in enumerate(component.bRepBodies):
            surfaces = []
            for face in body.faces:
                g = face.geometry
                record = {"type": g.objectType, "area_cm2": face.area}
                if g.objectType == adsk.core.Cylinder.classType():
                    record.update(origin_m=audit.vec(g.origin, .01), axis=audit.vec(g.axis), radius_m=g.radius * .01)
                elif g.objectType == adsk.core.Plane.classType():
                    record.update(origin_m=audit.vec(g.origin, .01), normal=audit.vec(g.normal))
                surfaces.append(record)
            calculator = body.meshManager.createMeshCalculator()
            calculator.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
            calculator.surfaceTolerance = .001  # cm: 0.01 mm for the comparison mesh
            calculator.maxNormalDeviation = 0.
            calculator.maxSideLength = .5  # cm: avoid long chords across the sculpted support surfaces
            mesh = calculator.calculate()
            nodes, indices = mesh.nodeCoordinatesAsDouble, mesh.nodeIndices
            triangles = [[[nodes[3*i+d] * .01 for d in range(3)] for i in indices[k:k+3]] for k in range(0, len(indices), 3)]
            import struct
            path = OUT / ("v2_native_body_%d.stl" % i)
            with path.open("wb") as stream:
                stream.write(b"Native v2 body audit only; meters".ljust(80, b" "))
                stream.write(struct.pack("<I", len(triangles)))
                for t in triangles:
                    stream.write(struct.pack("<12fH", *([0., 0., 0.] + sum(t, [])), 0))
            result["bodies"].append({"index": i, "name": body.name, "is_visible": body.isVisible,
                "is_solid": body.isSolid, "faces": body.faces.count,
                "properties": audit.props(body),
                "bbox_m": [audit.vec(body.boundingBox.minPoint, .01), audit.vec(body.boundingBox.maxPoint, .01)],
                "surface_types": dict(collections.Counter(s["type"] for s in surfaces)),
                "surfaces": surfaces, "audit_mesh": path.name,
                "mesh_surface_tolerance_mm": calculator.surfaceTolerance * 10., "mesh_triangles": len(triangles)})
        result["status"] = "passed"
        (OUT / "native_body_probe.json").write_text(json.dumps(result, indent=2))
        app.userInterface.messageBox("Native v2 bodies inspected.\n" + str(OUT / "native_body_probe.json"), "Native body probe complete")
    except Exception:
        result.update(status="failed", error=traceback.format_exc())
        (OUT / "native_body_probe.json").write_text(json.dumps(result, indent=2))
        app.userInterface.messageBox(result["error"], "Native body probe error")
    finally:
        if doc:
            doc.close(False)
        if previous:
            previous.activate()
