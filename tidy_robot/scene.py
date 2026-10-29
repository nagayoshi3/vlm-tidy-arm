# SPDX-License-Identifier: Apache-2.0
"""シーン定義 (scene.yaml) の読み込みと、物体の幾何情報。ROS 非依存。"""

import copy
from dataclasses import dataclass, field
from math import radians

import yaml

from tidy_robot.kinematics import polar_to_xy


@dataclass
class SceneObject:
    id: str
    name: str
    name_en: str
    shape: str
    color: list
    x: float
    y: float
    yaw: float                 # rad
    radius: float = 0.0
    length: float = 0.0
    size: list = field(default_factory=list)
    lying: bool = False
    mock: dict = field(default_factory=dict)
    state: str = 'pending'     # pending / keep / discard / done
    category: str = ''
    reason: str = ''

    @property
    def model_name(self):
        return f'tidy_{self.id}'

    @property
    def height(self):
        if self.shape == 'box':
            return self.size[2]
        if self.shape == 'sphere':
            return 2 * self.radius
        if self.shape == 'cylinder':
            return 2 * self.radius if self.lying else self.length
        raise ValueError(f'unknown shape {self.shape}')

    @property
    def footprint(self):
        """真上から見た大きさの目安 [m]（グリッパの開き量や描画に使う）。"""
        if self.shape == 'box':
            return max(self.size[0], self.size[1])
        if self.shape == 'sphere':
            return 2 * self.radius
        return self.length if self.lying else 2 * self.radius

    @property
    def center_z(self):
        """テーブル面基準の中心高さ。"""
        return self.height / 2.0

    def base_rpy(self):
        """SDF の姿勢 (roll, pitch, yaw)。横倒しの円柱は x 軸まわりに 90 度。"""
        roll = 1.5708 if (self.shape == 'cylinder' and self.lying) else 0.0
        return roll, 0.0, self.yaw


@dataclass
class Slot:
    x: float
    y: float
    yaw: float
    used_by: str = ''


@dataclass
class Zone:
    category: str
    label: str
    color: list
    slots: list


class Scene:

    def __init__(self, cfg):
        self.cfg = cfg
        w = cfg['world']
        self.world_name = w.get('name', 'default')
        self.table_z = float(w['table_z'])
        self.carry_z = float(w['carry_z'])
        self.approach_z = float(w['approach_z'])
        self.physics_step = float(w.get('physics_step', 0.004))   # 物理計算の刻み [s]。小さいほど重い
        self.camera = cfg['camera']
        self.instruction = cfg.get('instruction', '')

        self.objects = []
        for o in cfg['objects']:
            x, y = polar_to_xy(o['r'], o['theta_deg'])
            self.objects.append(SceneObject(
                id=o['id'], name=o['name'], name_en=o.get('name_en', o['id']),
                shape=o['shape'], color=o['color'], x=x, y=y,
                yaw=radians(o.get('yaw_deg', 0.0)),
                radius=o.get('radius', 0.0), length=o.get('length', 0.0),
                size=o.get('size', []), lying=o.get('lying', False),
                mock=o.get('mock', {})))

        self.zones = {}
        for z in cfg['zones']:
            slots = []
            for s in z['slots']:
                sx, sy = polar_to_xy(s['r'], s['theta_deg'])
                slots.append(Slot(sx, sy, radians(s.get('yaw_deg', s['theta_deg']))))
            self.zones[z['category']] = Zone(z['category'], z['label'], z['color'], slots)

        t = cfg['trash']
        tx, ty = polar_to_xy(t['r'], t['theta_deg'])
        self.trash = dict(t, x=tx, y=ty)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f))

    def copy(self):
        return Scene(copy.deepcopy(self.cfg))

    def get(self, obj_id):
        for o in self.objects:
            if o.id == obj_id:
                return o
        raise KeyError(obj_id)

    @property
    def categories(self):
        return list(self.zones.keys())

    def to_world(self, x, y, z):
        """土台基準 → ワールド座標（土台はワールド原点、テーブル面の高さ）。"""
        return x, y, z + self.table_z
