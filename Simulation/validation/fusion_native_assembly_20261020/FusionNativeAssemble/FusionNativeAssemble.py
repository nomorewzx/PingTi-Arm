"""Fusion Scripts & Add-Ins entry point for the native reassembly audit."""
import importlib.util
from pathlib import Path

def run(context):
    path = Path(__file__).resolve().parent.parent / "native_assemble.py"
    spec = importlib.util.spec_from_file_location("pingti_native_assembly", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.run(context)
