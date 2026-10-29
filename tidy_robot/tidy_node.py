# SPDX-License-Identifier: Apache-2.0
"""整理整頓ロボットのメインノード。

  認識   : 真上カメラ画像 (/tidy/camera/image) ＋ シーン状態（物体の ID・位置）
  判断   : VLM が「残す/捨てる・分類・順番」を決める（vlm.py）
  配置   : ルールで置き場と向きを決める（planner.py）
  実行   : 逆運動学で関節角を求め、JointTrajectory で CRANE+ V2 を動かす
  表示   : AI の判断と進行状況を /tidy/panel に画像で出す（動画用）

起動例:
  ros2 run tidy_robot tidy_node
  ros2 run tidy_robot tidy_node --ros-args -p instruction:="お客さんが来るので..."
"""

import array
import faulthandler
import json
import os
import signal
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:  # tf2 がない環境では順運動学で代用
    Buffer = TransformListener = None

from tidy_robot import kinematics as K
from tidy_robot.gz_pose import GzPose
from tidy_robot.planner import GRIP_INSET, check_reachable, make_tasks
from tidy_robot.scene import Scene
from tidy_robot.view import Panel, annotate_for_vlm, encode_jpeg
from tidy_robot.vlm import Judge, build_prompt


def _dur(sec):
    return DurationMsg(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))


def gripper_close_value(obj):
    """物体の幅に合わせたグリッパの閉じ量（見た目用。実機に移すなら要調整）。"""
    w = obj.footprint if not (obj.shape == 'cylinder' and obj.lying) else 2 * obj.radius
    if w >= 0.026:
        return -0.20
    if w >= 0.018:
        return -0.08
    return 0.05


class TidyNode(Node):

    def __init__(self):
        super().__init__('tidy_node')
        share = get_package_share_directory('tidy_robot')
        self.declare_parameter('scene_file', os.path.join(share, 'config', 'scene.yaml'))
        self.declare_parameter('instruction', '')
        self.declare_parameter('provider', '')
        self.declare_parameter('model', '')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('reach_tol', 0.10)
        self.declare_parameter('follow_source', 'tf')      # tf: Gazebo上の実際の手先 / fk: 順運動学
        self.declare_parameter('hand_frame', 'crane_plus_link_endtip')
        self.declare_parameter('grip_offset_xyz', [0.0, 0.0, 0.0])  # 手先フレーム基準の微調整 [m]
        self.declare_parameter('lead_extra', 0.05)   # 追従の先回り時間に足す秒数（遅れが残るなら増やす）
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('panel_scale', 1.0)   # 無圧縮パネルの縮小率（通信が詰まるなら 0.5）
        self.declare_parameter('judge_only', False)   # True: VLM の判断だけ行い、アームは動かさない
        self.declare_parameter('log_dir', os.path.expanduser('~/tidy_logs'))

        self.scene = Scene.load(self.get_parameter('scene_file').value)
        self.instruction = self.get_parameter('instruction').value or self.scene.instruction
        self.speed = max(0.2, float(self.get_parameter('speed').value))

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(String, '/tidy/status', 10)
        self.panel_pub = self.create_publisher(Image, '/tidy/panel', 2)
        # 無圧縮（1.4 MB/枚）は重い環境で通信が詰まるため、JPEG 版（50 KB 程度）も出す。
        # rqt_image_view では /tidy/panel/compressed を選ぶ
        self.panel_jpg_pub = self.create_publisher(CompressedImage, '/tidy/panel/compressed', 2)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 20)
        self.create_subscription(Image, '/tidy/camera/image', self.on_image, 2)

        self.lock = threading.Lock()
        self.joints = None
        self.joint_vel = None
        self.image = None
        self.gz = GzPose(self.scene.world_name, self.get_logger())
        self.tf_buffer = Buffer() if Buffer else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if Buffer else None
        self._tf_warned = False
        self.panel = Panel(self.scene)
        self.lines = ['整理整頓ロボット 起動中…']
        self.active_id = None
        self.carrying = None   # (obj, z_offset, yaw0, q1_0)
        self.running = True
        self.n_js = self.n_img = self.n_panel = 0   # 診断用カウンタ
        self._render_ms = 0.0
        # パネル描画は ROS のタイマーではなく専用スレッドで行う。描画が重くなっても
        # joint_states / カメラの受信コールバックを詰まらせないため（重い環境で発生）
        self.panel_thread = threading.Thread(target=self.panel_loop, daemon=True)
        self.panel_thread.start()
        self.follow_thread = threading.Thread(target=self.follow_loop, daemon=True)
        self.follow_thread.start()

    # ------------------------------------------------------------ callbacks
    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        self.n_js += 1
        if all(n in pos for n in K.ARM_JOINT_NAMES):
            with self.lock:
                self.joints = [pos[n] for n in K.ARM_JOINT_NAMES]
                self.joint_vel = [vel.get(n, 0.0) for n in K.ARM_JOINT_NAMES]

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            return
        ch = 4 if msg.encoding.endswith('a8') else 3
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // ch, ch)
        arr = arr[:, :msg.width, :3]
        if msg.encoding.startswith('bgr'):
            arr = arr[:, :, ::-1]
        self.n_img += 1
        with self.lock:
            self.image = np.ascontiguousarray(arr)

    def panel_loop(self):
        t_report = time.time()
        while self.running and rclpy.ok():
            t0 = time.time()
            try:
                self.publish_panel()
                self.n_panel += 1
            except Exception as e:   # 描画の失敗でノード全体を止めない
                self.get_logger().warn(f'パネル描画に失敗: {e!r}', throttle_duration_sec=5.0)
            self._render_ms = (time.time() - t0) * 1000
            if time.time() - t_report > 10:   # 10 秒ごとに受信・送信の状況を出す（診断用）
                t_report = time.time()
                self.get_logger().info(
                    f'診断: joint_states 受信 {self.n_js} 回, カメラ受信 {self.n_img} 回, '
                    f'パネル送信 {self.n_panel} 回, 描画 {self._render_ms:.0f} ms')
            time.sleep(max(0.1, 0.5 - (time.time() - t0)))

    def publish_panel(self):
        with self.lock:
            img = None if self.image is None else self.image.copy()
        out = self.panel.render(img, self.scene.objects, self.lines, self.active_id)
        stamp = self.get_clock().now().to_msg()
        jpg = encode_jpeg(out, 80)
        if jpg is not None:
            cm = CompressedImage()
            cm.header.stamp = stamp
            cm.format = 'jpeg'
            cm.data = array.array('B', jpg)
            self.panel_jpg_pub.publish(cm)
        scale = float(self.get_parameter('panel_scale').value)
        if scale < 0.99:
            import cv2
            out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        if scale < 0.99 or self.n_panel % 4 == 0:   # 等倍の無圧縮版は 0.5 Hz に間引く
            msg = Image()
            msg.header.stamp = stamp
            msg.height, msg.width = out.shape[0], out.shape[1]
            msg.encoding = 'rgb8'
            msg.step = msg.width * 3
            msg.data = array.array('B', out.tobytes())   # bytes を直接入れると 1 バイトずつ変換されて極端に遅い
            self.panel_pub.publish(msg)

    # ------------------------------------------------------------ status
    def say(self, *lines):
        self.lines = list(lines)
        text = ' / '.join(lines)
        self.status_pub.publish(String(data=text))
        print('\n'.join(['', '=' * 70] + [f'  {ln}' for ln in lines] + ['=' * 70]),
              flush=True)

    # ------------------------------------------------------------ motion
    def t(self, sec):
        return sec / self.speed

    def send_arm(self, points):
        """points: [(joints, time_from_start_sec), ...]"""
        msg = JointTrajectory()
        msg.joint_names = K.ARM_JOINT_NAMES
        for q, ts in points:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            p.time_from_start = _dur(ts)
            msg.points.append(p)
        self.arm_pub.publish(msg)

    def wait_reached(self, target, timeout, tol=None):
        tol = tol or float(self.get_parameter('reach_tol').value)
        t_end = time.time() + timeout
        err = None
        while time.time() < t_end and rclpy.ok():
            with self.lock:
                q = self.joints
            if q is not None:
                errs = [abs(a - b) for a, b in zip(q, target)]
                err = max(errs)
                if err < tol:
                    return True
            time.sleep(0.05)
        names = ['j1', 'j2', 'j3', 'j4']
        detail = '' if err is None else ' 残差(rad): ' + ', '.join(
            f'{n}={abs(a - b):.2f}' for n, a, b in zip(names, q, target))
        self.get_logger().warn(f'目標姿勢に到達する前にタイムアウトしました（続行します）{detail}')
        return False

    def move_joints(self, q, sec):
        sec = self.t(sec)
        self.send_arm([(q, sec)])
        self.wait_reached(q, sec * 3 + 5)

    def move_line(self, p0, p1, sec, n=6):
        """手先を真下向きのまま p0→p1 へ直線移動。"""
        sec = self.t(sec)
        pts = []
        for i in range(1, n + 1):
            a = i / n
            x, y, z = (p0[k] + (p1[k] - p0[k]) * a for k in range(3))
            q = K.ik_down(x, y, z)
            if q is None:
                raise RuntimeError(f'逆運動学の解なし: {x:.3f}, {y:.3f}, {z:.3f}')
            pts.append((q, sec * a))
        self.send_arm(pts)
        self.wait_reached(pts[-1][0], sec * 3 + 5)

    def move_above(self, x, y, z, sec):
        q = K.ik_down(x, y, z)
        if q is None:
            raise RuntimeError(f'逆運動学の解なし: {x:.3f}, {y:.3f}, {z:.3f}')
        self.move_joints(q, sec)

    def gripper(self, value, sec=0.8):
        sec = self.t(sec)
        msg = JointTrajectory()
        msg.joint_names = [K.GRIPPER_JOINT_NAME]
        p = JointTrajectoryPoint()
        p.positions = [float(value)]
        p.time_from_start = _dur(sec)
        msg.points = [p]
        self.grip_pub.publish(msg)
        time.sleep(sec + 0.3)

    # ------------------------------------------------------------ attach 方式の運搬
    def hand_position(self, q):
        """手先（指の間）の位置を crane_plus_base 基準で返す。tf が使えれば Gazebo 上の
        実際のモデルから、使えなければ順運動学で計算する。"""
        ox, oy, oz = [float(v) for v in self.get_parameter('grip_offset_xyz').value]
        if self.get_parameter('follow_source').value == 'tf' and self.tf_buffer is not None:
            try:
                tr = self.tf_buffer.lookup_transform(
                    'crane_plus_base', self.get_parameter('hand_frame').value,
                    rclpy.time.Time()).transform.translation
                # endtip は指先（launch で link4 から 0.09m）。指の間はその少し上
                return tr.x + ox, tr.y + oy, tr.z + GRIP_INSET + oz
            except TransformException as e:
                if not self._tf_warned:
                    self.get_logger().warn(f'tf が取れないため順運動学で追従します: {e}')
                    self._tf_warned = True
        x, y, z, _ = K.forward_kinematics(q)
        return x + ox, y + oy, z + GRIP_INSET + oz

    def follow_loop(self):
        while self.running:
            c = self.carrying
            with self.lock:
                q = self.joints
            if c is not None and q is not None:
                obj, _z_off, yaw0, q1_0 = c
                x, y, z = self.hand_position(q)
                # set_pose の遅れぶんだけ先回り：関節速度から手先の移動量を見積もって足す
                with self.lock:
                    v = self.joint_vel
                if v is not None:
                    lead = self.gz.latency + float(self.get_parameter('lead_extra').value)
                    q_pred = [a + b * lead for a, b in zip(q, v)]
                    px, py, pz, _ = K.forward_kinematics(q_pred)
                    cx, cy, cz, _ = K.forward_kinematics(q)
                    x, y, z = x + (px - cx), y + (py - cy), z + (pz - cz)
                obj.x, obj.y = x, y
                wx, wy, wz = self.scene.to_world(x, y, z)
                roll, pitch, _ = obj.base_rpy()
                self.gz.set_pose_async(obj.model_name, wx, wy, wz, roll, pitch,
                                       yaw0 + (q[0] - q1_0))
            time.sleep(0.03)

    def attach(self, obj, grasp_z):
        with self.lock:
            q1 = self.joints[0] if self.joints else 0.0
        # 物体の中心を指先より GRIP_INSET だけ上（指の間）に置く。指先が物の中心まで
        # 下りられない小物は、掴んだ瞬間に少し持ち上がって見える（許容）
        self.carrying = (obj, GRIP_INSET, obj.yaw, q1)

    def detach_and_place(self, obj, x, y, center_z, yaw):
        self.carrying = None
        self.gz.wait_idle()
        roll, pitch, _ = obj.base_rpy()
        wx, wy, wz = self.scene.to_world(x, y, center_z)
        self.gz.set_pose(obj.model_name, wx, wy, wz, roll, pitch, yaw)
        obj.x, obj.y, obj.yaw = x, y, yaw

    def drop_into_trash(self, obj, x, y, from_z, center_z):
        self.carrying = None
        self.gz.wait_idle()
        roll, pitch, _ = obj.base_rpy()
        for a in (0.35, 0.7, 1.0):   # 落下を数コマで表現
            z = from_z + (center_z - from_z) * a
            wx, wy, wz = self.scene.to_world(x, y, z)
            self.gz.set_pose(obj.model_name, wx, wy, wz, roll, pitch, obj.yaw)
        obj.x, obj.y = x, y

    # ------------------------------------------------------------ main sequence
    def wait_for(self, what, getter, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end and rclpy.ok():
            if getter() is not None:
                return True
            time.sleep(0.2)
        self.get_logger().warn(f'{what} が {timeout:.0f} 秒以内に届きませんでした')
        return False

    def save_log(self, log_dir, image_vlm, prompt, raw, plan, judge_label, elapsed):
        os.makedirs(log_dir, exist_ok=True)
        if image_vlm is not None:
            jpg = encode_jpeg(image_vlm)
            if jpg:
                with open(os.path.join(log_dir, 'vlm_input.jpg'), 'wb') as f:
                    f.write(jpg)
        with open(os.path.join(log_dir, 'prompt.txt'), 'w', encoding='utf-8') as f:
            f.write(prompt)
        with open(os.path.join(log_dir, 'vlm_raw.txt'), 'w', encoding='utf-8') as f:
            f.write(raw)
        with open(os.path.join(log_dir, 'plan.json'), 'w', encoding='utf-8') as f:
            json.dump({'judge': judge_label, 'elapsed_sec': round(elapsed, 2),
                       'instruction': self.instruction, 'plan': plan},
                      f, ensure_ascii=False, indent=2)

    def run(self):
        sc = self.scene
        self.say('整理整頓ロボット', 'ロボットの起動を待っています…')
        if not self.wait_for('/joint_states', lambda: self.joints, 120):
            self.say('エラー', '/joint_states が来ません。Gazebo とコントローラを確認してください')
            return
        self.gripper(K.GRIPPER_OPEN)
        self.move_joints(K.HOME_JOINTS, 3.0)
        self.wait_for('カメラ画像 /tidy/camera/image', lambda: self.image, 30)
        time.sleep(float(self.get_parameter('start_delay').value))

        # ---- 1. 認識 → 判断（VLM）
        judge = Judge(self.get_parameter('provider').value or None,
                      self.get_parameter('model').value or None,
                      log=lambda m: self.get_logger().warn(m))
        self.say(f'指示：{self.instruction}',
                 f'AI（{judge.label}）が机の上を見て、片付け方を考えています…')
        with self.lock:
            img = None if self.image is None else self.image.copy()
        img_vlm = annotate_for_vlm(sc, img)
        plan, raw, elapsed = judge.decide(sc, self.instruction, encode_jpeg(img_vlm))
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.join(self.get_parameter('log_dir').value, stamp)
        self.save_log(log_dir, img_vlm, build_prompt(sc, self.instruction), raw, plan,
                      judge.label, elapsed)
        self.get_logger().info(f'VLM の判断を保存しました: {log_dir}')

        items = {it['id']: it for it in plan['items']}
        for o in sc.objects:
            it = items[o.id]
            o.state, o.category, o.reason = it['decision'], it['category'], it['reason']
        n_keep = sum(1 for it in plan['items'] if it['decision'] == 'keep')
        n_disc = sum(1 for it in plan['items'] if it['decision'] == 'discard')
        n_leave = len(plan['items']) - n_keep - n_disc
        self.say(f'AIの判断：{plan["scene_summary"]}',
                 f'片付ける {n_keep} 個・捨てる {n_disc} 個・そのまま {n_leave} 個'
                 f'（{elapsed:.1f} 秒で判断）',
                 f'順番：{plan["order_reason"]}')
        marks = {'keep': '片付ける', 'discard': '捨てる', 'leave': 'そのまま'}
        for it in plan['items']:
            o = sc.get(it['id'])
            mark = marks[it['decision']]
            print(f'    {o.id} {o.name:<8} → {mark:<3} [{it["category"]}] {it["reason"]}')
        if self.get_parameter('judge_only').value:
            self.say('判断のみモード：アームは動かしません', f'記録: {log_dir}')
            return
        time.sleep(self.t(4.0))

        # ---- 2. 配置先決定（ルール）
        tasks = make_tasks(sc, plan)
        bad = check_reachable(sc, tasks)
        if bad:
            self.get_logger().error(f'手が届かない姿勢があります: {bad}')
        bad_ids = {b[0] for b in bad}

        # ---- 3. 実行
        for i, tk in enumerate(tasks, 1):
            if tk.obj_id in bad_ids:
                continue
            o = sc.get(tk.obj_id)
            verb = '捨てる' if tk.drop else '片付ける'
            self.active_id = o.id
            self.say(f'[{i}/{len(tasks)}] {o.name} → {tk.dest_label} に{verb}',
                     f'理由：{tk.reason}')
            try:
                self.pick_and_place(o, tk)
            except RuntimeError as e:
                self.get_logger().error(str(e))
                self.carrying = None
            o.state = 'done'
            self.active_id = None

        self.move_joints(K.HOME_JOINTS, 3.0)
        self.say('整理整頓 完了！',
                 f'指示：{self.instruction}',
                 f'{n_keep} 個を分類ごとに揃えて配置、ゴミ {n_disc} 個を処分、'
                 f'{n_leave} 個は使うのでそのまま')

    def pick_and_place(self, o, tk):
        sc = self.scene
        above_pick = (tk.pick_x, tk.pick_y, sc.approach_z)
        at_pick = (tk.pick_x, tk.pick_y, tk.grasp_z)
        above_place = (tk.place_x, tk.place_y, sc.carry_z)
        at_place = (tk.place_x, tk.place_y, tk.release_z)

        self.move_above(*above_pick, 2.5)
        self.move_line(above_pick, at_pick, 1.5)
        self.gripper(gripper_close_value(o))
        self.attach(o, tk.grasp_z)
        self.move_line(at_pick, (tk.pick_x, tk.pick_y, sc.carry_z), 1.5)
        self.move_above(*above_place, 3.0)
        if tk.drop:
            self.gripper(K.GRIPPER_OPEN, 0.5)
            self.drop_into_trash(o, tk.place_x, tk.place_y,
                                 sc.carry_z + o.center_z - tk.grasp_z, tk.place_center_z)
            return
        self.move_line(above_place, at_place, 1.5)
        self.gripper(K.GRIPPER_OPEN, 0.6)
        self.detach_and_place(o, tk.place_x, tk.place_y, tk.place_center_z, tk.place_yaw)
        self.move_line(at_place, above_place, 1.2)


def main():
    faulthandler.register(signal.SIGUSR1, all_threads=True)   # kill -USR1 <pid> で全スレッドのスタックを表示
    rclpy.init()
    node = TidyNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
        if node.get_parameter('judge_only').value:
            time.sleep(1.0)     # パネルの最終描画を待ってから終了
        else:
            print('\nCtrl+C で終了（パネル表示は続きます）', flush=True)
            while rclpy.ok():
                time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n終了します', flush=True)
    finally:
        node.running = False
        try:
            node.gz.wait_idle(1.0)
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass
        os._exit(0)     # 残ったスレッドの join で固まらないよう、ここで確実に終了する


if __name__ == '__main__':
    main()
