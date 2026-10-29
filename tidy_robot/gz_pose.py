# SPDX-License-Identifier: Apache-2.0
"""Ignition Gazebo の set_pose サービスで物体を動かす（attach 方式の運搬に使う）。

Fortress の ros_gz_bridge は set_pose サービスを中継できないため、
`ign service` コマンドを呼ぶ。追従中は前の呼び出しが終わってから次を送る
（終わっていなければその周期は飛ばす）ので、環境の速さに合わせて自動で
更新レートが決まる。
"""

import math
import shutil
import subprocess


def yaw_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_request(name, x, y, z, roll, pitch, yaw):
    qx, qy, qz, qw = yaw_to_quat(roll, pitch, yaw)
    return (f'name: "{name}", position: {{x: {x:.5f}, y: {y:.5f}, z: {z:.5f}}}, '
            f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}')


class GzPose:

    def __init__(self, world_name='default', logger=None):
        self.service = f'/world/{world_name}/set_pose'
        self.cli = shutil.which('ign') or shutil.which('gz') or 'ign'
        self.logger = logger
        self._inflight = None
        self._t_start = None
        self.latency = 0.2      # set_pose 1回の所要時間（実測の移動平均, 秒）

    def _cmd(self, req):
        return [self.cli, 'service', '-s', self.service,
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000', '--req', req]

    def set_pose(self, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """同期呼び出し。物体を最終位置に置くときに使う。"""
        req = pose_request(name, x, y, z, roll, pitch, yaw)
        for _ in range(3):
            r = subprocess.run(self._cmd(req), capture_output=True, text=True, timeout=10)
            if 'true' in r.stdout:
                return True
        if self.logger:
            self.logger.warn(f'set_pose failed for {name}: {r.stdout} {r.stderr}')
        return False

    def set_pose_async(self, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """非同期呼び出し。前の要求が終わっていなければ何もしない。"""
        import time
        if self._inflight is not None:
            if self._inflight.poll() is None:
                return False
            if self._t_start is not None:
                self.latency = 0.8 * self.latency + 0.2 * (time.time() - self._t_start)
        req = pose_request(name, x, y, z, roll, pitch, yaw)
        self._t_start = time.time()
        self._inflight = subprocess.Popen(
            self._cmd(req), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True

    def wait_idle(self, timeout=3.0):
        if self._inflight is not None:
            try:
                self._inflight.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self._inflight.kill()
