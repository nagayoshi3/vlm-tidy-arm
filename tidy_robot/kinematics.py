# Copyright 2026 tidy_robot contributors
# SPDX-License-Identifier: Apache-2.0
#
# CRANE+ V2 の順運動学・逆運動学。
# 『ROS 2とPythonで作って学ぶAIロボット入門 改訂第2版』chapter6 の
# crane_plus_commander/kinematics.py (Apache-2.0) をもとに、整理整頓タスク用の
# ヘルパ（真上からの把持姿勢、ワールド座標との変換）を追加したもの。
# 座標系はすべて crane_plus_base 基準（ロボット土台の中心、テーブル面の高さ）。

from math import acos, atan2, cos, pi, sin, sqrt

RAD = pi / 180.0
JOINT_MIN = [-150 * RAD, -104.55 * RAD, -146.48 * RAD, -105.43 * RAD]
JOINT_MAX = [150.0 * RAD, 102.2 * RAD, 140.62 * RAD, 101.32 * RAD]
GRIPPER_MIN = -40.62 * RAD + 0.001
GRIPPER_MAX = 38.27 * RAD - 0.001
# physicalai_teleop と同じ値（負が開く方向）
GRIPPER_OPEN = -0.5236
GRIPPER_CLOSE = 0.1745

L1 = 0.0456 + 0.026
L2 = 0.083
L3 = 0.0935
L4 = 0.045 + 0.045

PITCH_DOWN = -pi / 2  # 手先を真下に向ける

# アームを畳んだ待機姿勢（書籍 commander5 と同じ）
HOME_JOINTS = [0.0, -1.16, -2.01, -0.73]

ARM_JOINT_NAMES = [
    'crane_plus_joint1',
    'crane_plus_joint2',
    'crane_plus_joint3',
    'crane_plus_joint4',
]
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'


def joint_in_range(joint):
    return [lo <= q <= hi for q, lo, hi in zip(joint, JOINT_MIN, JOINT_MAX)]


def normalize_angle(x):
    while x < -pi:
        x += 2 * pi
    while x > pi:
        x -= 2 * pi
    return x


def forward_kinematics(joint):
    q1, q2, q3, q4 = joint
    q2 = pi / 2 - q2
    r = L2 * cos(q2) + L3 * cos(q2 + q3) + L4 * cos(q2 + q3 + q4)
    x = r * cos(q1)
    y = r * sin(q1)
    z = L1 + L2 * sin(q2) + L3 * sin(q2 + q3) + L4 * sin(q2 + q3 + q4)
    pitch = q2 + q3 + q4
    return [x, y, z, pitch]


def inverse_kinematics(endtip, elbow_up):
    x, y, z, pitch = endtip
    r = sqrt(x ** 2 + y ** 2)
    if r == 0.0:
        return None
    q1 = atan2(y, x)
    rr = r - L4 * cos(pitch)
    zz = z - L4 * sin(pitch)
    d = sqrt(rr ** 2 + (zz - L1) ** 2)
    cq3 = (d ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
    if abs(cq3) > 1.0:
        return None
    phi = atan2(zz - L1, rr)
    cpsi = (d ** 2 + L2 ** 2 - L3 ** 2) / (2 * L2 * d)
    if abs(cpsi) > 1.0:
        return None
    psi = acos(cpsi)
    if elbow_up:
        q2 = normalize_angle(phi + psi)
        q3 = -acos(cq3)
    else:
        q2 = normalize_angle(phi - psi)
        q3 = acos(cq3)
    q4 = normalize_angle(pitch - q2 - q3)
    q2 = pi / 2 - q2
    return [q1, q2, q3, q4]


def ik_down(x, y, z):
    """手先を真下に向けて (x, y, z) に置く関節角。届かなければ None。"""
    for elbow_up in (True, False):
        joint = inverse_kinematics([x, y, z, PITCH_DOWN], elbow_up)
        if joint is not None and all(joint_in_range(joint)):
            return joint
    return None


def polar_to_xy(r, theta_deg):
    th = theta_deg * RAD
    return r * cos(th), r * sin(th)
