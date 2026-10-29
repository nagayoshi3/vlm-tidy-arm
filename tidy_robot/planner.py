# SPDX-License-Identifier: Apache-2.0
"""配置先決定ノードの中身：VLM の判断（意味）をロボットが実行できる座標（幾何）に落とす。

意味の判断は AI、幾何はルール、と役割を分けている。
  - keep  : 分類ごとの置き場（slot）に、前から順に詰めて、向きを揃えて置く（整頓）
  - discard: ゴミ箱の中に、重ならないように少しずつずらして落とす（整理）
  - leave : 今使う物。動かさない（タスクを作らない）
ROS 非依存。
"""

from dataclasses import dataclass
from math import pi

from tidy_robot.kinematics import ik_down

GRASP_MIN_Z = 0.03        # 指先（手先）をこれより下げない。指の形がこれより下にあり、机に当たるため
GRIP_INSET = 0.012        # 掴んだ物の中心は、指先よりこれだけ上（指の間）に見せる
TRASH_OFFSETS = [(0.0, 0.0), (0.012, 0.010), (-0.012, -0.010),
                 (0.012, -0.012), (-0.012, 0.012), (0.0, 0.015)]


@dataclass
class Task:
    obj_id: str
    name: str
    decision: str
    category: str
    dest_label: str
    reason: str
    pick_x: float
    pick_y: float
    grasp_z: float          # 掴むときの手先高さ（テーブル面基準）
    place_x: float
    place_y: float
    place_center_z: float   # 置いた後の物体中心高さ
    place_yaw: float
    release_z: float        # 手を開く手先高さ
    drop: bool              # True ならゴミ箱に落とす


def final_yaw(obj, slot_yaw):
    # 横倒しの円柱（ペン）は軸を放射方向に揃える
    if obj.shape == 'cylinder' and obj.lying:
        return slot_yaw - pi / 2
    return slot_yaw


def make_tasks(scene, plan):
    items = {it['id']: it for it in plan['items']}
    tasks = []
    trash_count = 0
    t = scene.trash
    for oid in plan['order']:
        it = items[oid]
        if it['decision'] == 'leave':
            continue            # 今使う物は動かさない
        o = scene.get(oid)
        grasp_z = max(o.center_z - GRIP_INSET, GRASP_MIN_Z)
        if it['decision'] == 'discard':
            dx, dy = TRASH_OFFSETS[trash_count % len(TRASH_OFFSETS)]
            trash_count += 1
            tasks.append(Task(
                oid, o.name, 'discard', 'trash', t['label'], it['reason'],
                o.x, o.y, grasp_z,
                t['x'] + dx, t['y'] + dy, 0.003 + o.center_z + 0.004 * (trash_count - 1),
                o.yaw, scene.approach_z, True))
            continue
        zone = scene.zones[it['category']]
        slot = next((s for s in zone.slots if not s.used_by), None)
        if slot is None:
            raise RuntimeError(f'{zone.label} の置き場が足りません')
        slot.used_by = oid
        tasks.append(Task(
            oid, o.name, 'keep', zone.category, zone.label, it['reason'],
            o.x, o.y, grasp_z,
            slot.x, slot.y, o.center_z, final_yaw(o, slot.yaw),
            grasp_z, False))
    return tasks


def check_reachable(scene, tasks):
    """全タスクの姿勢が逆運動学で解けるか確認し、解けないものを返す。"""
    bad = []
    for tk in tasks:
        for label, (x, y, z) in {
            'pick': (tk.pick_x, tk.pick_y, tk.grasp_z),
            'pick_above': (tk.pick_x, tk.pick_y, scene.approach_z),
            'place': (tk.place_x, tk.place_y, tk.release_z),
            'place_above': (tk.place_x, tk.place_y, scene.approach_z),
        }.items():
            if ik_down(x, y, z) is None:
                bad.append((tk.obj_id, label, round(x, 3), round(y, 3), z))
    return bad
