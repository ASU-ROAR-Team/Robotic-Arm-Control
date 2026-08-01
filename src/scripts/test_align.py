#!/usr/bin/env python3
"""Standalone test harness for align_link6_z_to_semantic logic.

This script replicates the quaternion/vector helpers from teleop.py
and runs several scenarios to verify that aligning LINK_6 Z to semantic
directions produces the expected resulting Z axis.
"""
import math


def quat_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def euler_from_quat(qx: float, qy: float, qz: float, qw: float):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def rotate_vector(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    ix =  w * vx + y * vz - z * vy
    iy =  w * vy + z * vx - x * vz
    iz =  w * vz + x * vy - y * vx
    iw = -x * vx - y * vy - z * vz
    rx = ix * w + iw * -x + iy * -z - iz * -y
    ry = iy * w + iw * -y + iz * -x - ix * -z
    rz = iz * w + iw * -z + ix * -y - iy * -x
    return (rx, ry, rz)


def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (qx, qy, qz, qw)


def quat_from_two_vectors(v_from, v_to):
    fx, fy, fz = v_from
    tx, ty, tz = v_to
    fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
    tmag = math.sqrt(tx * tx + ty * ty + tz * tz)
    if fmag < 1e-9 or tmag < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    v1 = (fx / fmag, fy / fmag, fz / fmag)
    v2 = (tx / tmag, ty / tmag, tz / tmag)
    dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    if dot >= 1.0 - 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if dot <= -1.0 + 1e-12:
        if abs(v1[0]) < abs(v1[1]):
            ort = (1.0, 0.0, 0.0)
        else:
            ort = (0.0, 1.0, 0.0)
        cx = v1[1] * ort[2] - v1[2] * ort[1]
        cy = v1[2] * ort[0] - v1[0] * ort[2]
        cz = v1[0] * ort[1] - v1[1] * ort[0]
        mag = math.sqrt(cx * cx + cy * cy + cz * cz)
        if mag < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        sx, sy, sz = cx / mag, cy / mag, cz / mag
        return (sx, sy, sz, 0.0)
    cx = v1[1] * v2[2] - v1[2] * v2[1]
    cy = v1[2] * v2[0] - v1[0] * v2[2]
    cz = v1[0] * v2[1] - v1[1] * v2[0]
    s = math.sqrt((1.0 + dot) * 2.0)
    invs = 1.0 / s
    qx = cx * invs
    qy = cy * invs
    qz = cz * invs
    qw = 0.5 * s
    return (qx, qy, qz, qw)


SEMANTIC_MAP = {
    'Forward': (-1.0, 0.0, 0.0),
    'Backward': (1.0, 0.0, 0.0),
    'Right': (0.0, 1.0, 0.0),
    'Left': (0.0, -1.0, 0.0),
    'Up': (0.0, 0.0, 1.0),
    'Down': (0.0, 0.0, -1.0),
}


def align_simulation(q_current, semantic):
    # local Z axis of LINK_6
    local_z = (0.0, 0.0, 1.0)
    cur_z = rotate_vector(q_current, local_z)
    desired = SEMANTIC_MAP[semantic]
    rot_q = quat_from_two_vectors(cur_z, desired)
    q_new = quat_mult(rot_q, q_current)
    new_z = rotate_vector(q_new, local_z)
    rpy = euler_from_quat(q_new[0], q_new[1], q_new[2], q_new[3])
    return {
        'semantic': semantic,
        'cur_z': cur_z,
        'desired': desired,
        'rot_q': rot_q,
        'q_new': q_new,
        'new_z': new_z,
        'rpy_deg': tuple(math.degrees(a) for a in rpy),
    }


def approx(v):
    return tuple(round(x, 4) for x in v)


def run_tests():
    tests = []
    # identity orientation
    tests.append(('identity', quat_from_euler(0.0, 0.0, 0.0)))
    # rotated 90 deg about X
    tests.append(('rot_x_90', quat_from_euler(math.radians(90), 0.0, 0.0)))
    # rotated 90 deg about Y
    tests.append(('rot_y_90', quat_from_euler(0.0, math.radians(90), 0.0)))
    # rotated 90 deg about Z
    tests.append(('rot_z_90', quat_from_euler(0.0, 0.0, math.radians(90))))

    for name, q in tests:
        print(f"\n--- Test: {name} q={q} ---")
        for sem in SEMANTIC_MAP:
            out = align_simulation(q, sem)
            print(f"{sem}: cur_z={approx(out['cur_z'])} -> desired={out['desired']} -> new_z={approx(out['new_z'])} rpy_deg={tuple(round(x,1) for x in out['rpy_deg'])}")


if __name__ == '__main__':
    run_tests()
