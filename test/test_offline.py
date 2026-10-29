# SPDX-License-Identifier: Apache-2.0
"""ROS なしで回せるテスト（届くか・計画・VLM 出力の検証・SDF・投影）。
   cd tidy_robot && python3 -m pytest -q test/test_offline.py
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import pytest

from tidy_robot import kinematics as K
from tidy_robot.planner import check_reachable, make_tasks
from tidy_robot.scene import Scene
from tidy_robot.view import project
from tidy_robot.vlm import JudgeError, extract_json, mock_plan, normalize_plan
from tidy_robot.world_gen import build_world

SCENE = os.path.join(os.path.dirname(__file__), '..', 'config', 'scene.yaml')


@pytest.fixture
def scene():
    return Scene.load(SCENE)


def test_fk_ik_roundtrip():
    q = K.ik_down(0.12, 0.05, 0.03)
    x, y, z, pitch = K.forward_kinematics(q)
    assert abs(x - 0.12) < 1e-6 and abs(y - 0.05) < 1e-6 and abs(z - 0.03) < 1e-6
    assert abs(pitch - K.PITCH_DOWN) < 1e-6


def test_home_in_range():
    assert all(K.joint_in_range(K.HOME_JOINTS))


def test_objects_do_not_overlap(scene):
    pts = [(o.id, o.x, o.y, 0.05 if (o.shape == 'cylinder' and o.lying) else o.footprint)
           for o in scene.objects]
    for o in scene.zones.values():
        pts += [(f'slot_{o.category}', s.x, s.y, 0.034) for s in o.slots]
    t = scene.trash
    pts.append(('trash', t['x'], t['y'], 2 * t['radius'] * 1.42))
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            a, b = pts[i], pts[j]
            d = math.hypot(a[1] - b[1], a[2] - b[2])
            assert d > (a[3] + b[3]) / 2 * 0.95, f'{a[0]} と {b[0]} が近すぎる: {d:.3f}'
    # ロボットの土台（半径約 4cm）に重ならない
    for p in pts:
        assert math.hypot(p[1], p[2]) - p[3] / 2 > 0.04, p[0]


def test_mock_plan_reachable_and_same_elbow(scene):
    plan = mock_plan(scene)
    tasks = make_tasks(scene, plan)
    assert check_reachable(scene, tasks) == []
    # 全姿勢を elbow_up=True で解ける（途中で肘が反転しない）
    for tk in tasks:
        for x, y, z in [(tk.pick_x, tk.pick_y, tk.grasp_z),
                        (tk.pick_x, tk.pick_y, scene.carry_z),
                        (tk.place_x, tk.place_y, scene.carry_z),
                        (tk.place_x, tk.place_y, tk.release_z)]:
            q = K.inverse_kinematics([x, y, z, K.PITCH_DOWN], True)
            assert q is not None and all(K.joint_in_range(q)), (tk.obj_id, x, y, z)


def test_every_category_assignment_reachable(scene):
    """VLM がどの物をどの分類にしても届くこと（全組合せの代表）。"""
    for cat in scene.categories + ['trash']:
        sc = Scene.load(SCENE)
        items = [{'id': o.id, 'decision': 'discard' if cat == 'trash' else 'keep',
                  'category': cat, 'reason': ''} for o in sc.objects]
        plan = normalize_plan({'items': items, 'order': []}, sc)
        tasks = make_tasks(sc, plan)
        assert check_reachable(sc, tasks) == [], cat


def test_normalize_plan(scene):
    raw = extract_json('```json\n' + json.dumps({
        'scene_summary': 's',
        'items': [{'id': o.id, 'decision': 'keep', 'category': 'unknown_cat',
                   'reason': 'r'} for o in scene.objects],
        'order': ['obj3', 'obj3', 'objX'],
        'order_reason': 'x'}) + '\n```')
    plan = normalize_plan(raw, scene)
    assert plan['order'][0] == 'obj3'
    assert sorted(plan['order']) == sorted(o.id for o in scene.objects)
    cats = [it['category'] for it in plan['items']]
    assert cats.count('other') == 2  # 不明な分類は other へ、あふれた分は空きのある置き場へ
    assert set(cats) <= set(scene.categories)
    with pytest.raises(JudgeError):
        normalize_plan({'items': [], 'order': []}, scene)
    with pytest.raises(JudgeError):
        extract_json('no json here')


def test_world_sdf_is_valid_xml(scene):
    root = ET.fromstring(build_world(scene))
    names = [m.get('name') for m in root.iter('model')]
    for o in scene.objects:
        assert o.model_name in names
    assert 'tidy_overhead_camera' in names


def test_projection_layout(scene):
    w, h = scene.camera['width'], scene.camera['height_px']
    for o in scene.objects:
        u, v = project(scene, o.x, o.y, o.height)
        assert 0 <= u < w and 0 <= v < h, o.id
    # 左（+y）にある物は画像の左側、奥（+x）にある物は画像の上側
    assert project(scene, 0.1, 0.1)[0] < project(scene, 0.1, -0.1)[0]
    assert project(scene, 0.15, 0)[1] < project(scene, 0.05, 0)[1]
    t = scene.trash
    u, v = project(scene, t['x'], t['y'])
    assert 0 <= u < w and 0 <= v < h
