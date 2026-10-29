# SPDX-License-Identifier: Apache-2.0
"""scene.yaml から Ignition Gazebo (Fortress) 用のワールド SDF を生成する。ROS 非依存。

机の上の物体は static（物理計算の対象外）の小物として置き、掴んでいる間は
tidy_node が set_pose サービスで手先に追従させる（Fortress では link の
gravity/kinematic 指定が効かず落下したため static にしている）（把持の物理計算を省いた
"attach 方式"）。シミュレータの把持は滑りやすく、2 週間の課題で AI の判断部分に
集中するための割り切り。README とレポートにその旨を明記すること。
"""

from math import pi

from tidy_robot.scene import Scene

FUEL = 'https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models'


def _rgba(c):
    return ' '.join(f'{v:.3f}' for v in c)


def _material(color):
    return (f'<material><ambient>{_rgba(color)}</ambient>'
            f'<diffuse>{_rgba(color)}</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular></material>')


def _geometry(o):
    if o.shape == 'box':
        sx, sy, sz = o.size
        return f'<box><size>{sx} {sy} {sz}</size></box>'
    if o.shape == 'sphere':
        return f'<sphere><radius>{o.radius}</radius></sphere>'
    if o.shape == 'cylinder':
        return (f'<cylinder><radius>{o.radius}</radius>'
                f'<length>{o.length}</length></cylinder>')
    raise ValueError(o.shape)


def object_model(scene, o):
    x, y, z = scene.to_world(o.x, o.y, o.center_z)
    roll, pitch, yaw = o.base_rpy()
    return f'''
    <model name="{o.model_name}">
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <static>true</static>
      <link name="link">
        <inertial><mass>0.02</mass>
          <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry>{_geometry(o)}</geometry>
          {_material(o.color)}
        </visual>
      </link>
    </model>'''


def _static_box(name, x, y, z, sx, sy, sz, color, yaw=0.0):
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          {_material(color)}
        </visual>
      </link>
    </model>'''


def zone_models(scene):
    out = []
    for zone in scene.zones.values():
        for i, s in enumerate(zone.slots):
            x, y, z = scene.to_world(s.x, s.y, 0.001)
            out.append(f'''
    <model name="tidy_mat_{zone.category}_{i}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.017</radius><length>0.002</length></cylinder></geometry>
          {_material(zone.color)}
        </visual>
      </link>
    </model>''')
    return ''.join(out)


def trash_model(scene):
    t = scene.trash
    a = t['radius']
    h = t['wall_height']
    th = 0.003
    x, y, _ = scene.to_world(t['x'], t['y'], 0.0)
    z0 = scene.table_z
    c = t['color']
    yaw = 0.0
    parts = [
        _static_box('tidy_trash_bottom', x, y, z0 + th / 2, 2 * a, 2 * a, th, c),
        _static_box('tidy_trash_wall_n', x + a, y, z0 + h / 2, th, 2 * a, h, c, yaw),
        _static_box('tidy_trash_wall_s', x - a, y, z0 + h / 2, th, 2 * a, h, c, yaw),
        _static_box('tidy_trash_wall_e', x, y + a, z0 + h / 2, 2 * a, th, h, c, yaw),
        _static_box('tidy_trash_wall_w', x, y - a, z0 + h / 2, 2 * a, th, h, c, yaw),
    ]
    return ''.join(parts)


def camera_model(scene):
    cam = scene.camera
    z = scene.table_z + cam['height']
    return f'''
    <model name="tidy_overhead_camera">
      <static>true</static>
      <pose>{cam['x']} {cam['y']} {z:.4f} 0 {pi / 2:.5f} 0</pose>
      <link name="link">
        <sensor name="overhead" type="camera">
          <camera>
            <horizontal_fov>{cam['hfov']}</horizontal_fov>
            <image><width>{cam['width']}</width><height>{cam['height_px']}</height></image>
            <clip><near>0.05</near><far>5.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{cam['update_rate']}</update_rate>
          <visualize>false</visualize>
          <topic>tidy/camera/image</topic>
        </sensor>
      </link>
    </model>'''


def build_world(scene: Scene) -> str:
    objects = ''.join(object_model(scene, o) for o in scene.objects)
    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{scene.world_name}">
    <physics name="step" type="ignored">
      <max_step_size>{scene.physics_step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <include><uri>{FUEL}/Sun</uri></include>
    <include><uri>{FUEL}/Ground%20Plane</uri></include>
    <include>
      <uri>{FUEL}/Table</uri>
      <pose>0.29 0 0 0 0 1.5708</pose>
    </include>
{zone_models(scene)}
{trash_model(scene)}
{camera_model(scene)}
{objects}
  </world>
</sdf>
'''


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('scene')
    p.add_argument('out')
    a = p.parse_args()
    with open(a.out, 'w', encoding='utf-8') as f:
        f.write(build_world(Scene.load(a.scene)))


if __name__ == '__main__':
    main()
