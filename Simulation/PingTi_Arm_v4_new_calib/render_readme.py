"""Render the actual strengthened URDF for the README using PyBullet on CPU."""
import math
from pathlib import Path
import struct
import zlib

import numpy as np
import pybullet as p

OUT = Path(__file__).resolve().parent
DESTINATION = OUT.parents[1] / "media/pingti_arm_render.png"


def write_png(path, rgba):
    height, width = rgba.shape[:2]
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)
    rows = b"".join(b"\x00" + row.tobytes() for row in rgba)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(rows, 9)) + chunk(b"IEND", b""))


def main():
    connection = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(str(OUT / "pingti_arm_v4_new_calib.urdf"), useFixedBase=True,
                           flags=p.URDF_USE_INERTIA_FROM_FILE)
        for i in range(-1, p.getNumJoints(robot)):
            name = "base_link" if i == -1 else p.getJointInfo(robot, i)[12].decode()
            servo = name.startswith("sts")
            color = [.13, .16, .20, 1.] if servo else [.65, .73, .79, 1.]
            p.changeVisualShape(robot, i, rgbaColor=color, specularColor=[.18, .18, .18])
            if i >= 0:
                joint = p.getJointInfo(robot, i)[1].decode()
                p.resetJointState(robot, i, math.radians(22.) if joint == "gripper" else 0.)
        width, height, samples = 1600, 1200, 3
        view = p.computeViewMatrix([.95, -1.10, .82], [.27, 0., .29], [0., 0., 1.])
        projection = p.computeProjectionMatrixFOV(22., width / height, .01, 5.)
        frame = p.getCameraImage(width * samples, height * samples, viewMatrix=view,
                                 projectionMatrix=projection, renderer=p.ER_TINY_RENDERER,
                                 lightDirection=[-1.5, -2., 5.], lightColor=[1., 1., 1.],
                                 lightAmbientCoeff=.45, lightDiffuseCoeff=.55, lightSpecularCoeff=.10,
                                 shadow=0)
        pixels = np.asarray(frame[2], dtype=np.uint8).reshape(height * samples, width * samples, 4)
        pixels = np.rint(pixels.reshape(height, samples, width, samples, 4).mean(axis=(1, 3))).astype(np.uint8)
        write_png(DESTINATION, pixels)
        print(str(DESTINATION))
    finally:
        p.disconnect(connection)


if __name__ == "__main__":
    main()
