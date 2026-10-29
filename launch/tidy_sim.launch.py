# SPDX-License-Identifier: Apache-2.0
# crane_plus_gazebo/no_moveit_crane_plus_with_table.launch.py (Apache-2.0, RT Corporation)
# をもとに、整理整頓用のワールド（scene.yaml から生成）と真上カメラの bridge を追加したもの。

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from tidy_robot.scene import Scene
from tidy_robot.world_gen import build_world

# 動画映えする Gazebo の視点（x y z roll pitch yaw）。好みで調整してください
GUI_CAMERA_POSE = '0.26 -0.17 1.18 0 0.40 2.62'


def _write_files(context):
    scene_file = LaunchConfiguration('scene_file').perform(context)
    scene = Scene.load(scene_file)
    out_dir = os.path.join(tempfile.gettempdir(), 'tidy_robot')
    os.makedirs(out_dir, exist_ok=True)
    world_file = os.path.join(out_dir, 'tidy_desk.sdf')
    with open(world_file, 'w', encoding='utf-8') as f:
        f.write(build_world(scene))

    src_gui = os.path.join(get_package_share_directory('crane_plus_gazebo'), 'gui', 'gui.config')
    with open(src_gui, encoding='utf-8') as f:
        gui = f.read()
    gui = re.sub(r'<camera_pose>.*?</camera_pose>',
                 f'<camera_pose>{GUI_CAMERA_POSE}</camera_pose>', gui)
    gui_file = os.path.join(out_dir, 'gui.config')
    with open(gui_file, 'w', encoding='utf-8') as f:
        f.write(gui)

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('crane_plus_description'))}
    # -r で起動時にシミュレーションを開始しないとコントローラが起動しない
    return [ExecuteProcess(
        cmd=['ign gazebo -r', world_file, '--gui-config', gui_file],
        output='screen', additional_env=env, shell=True)]


def generate_launch_description():
    default_scene = os.path.join(
        get_package_share_directory('tidy_robot'), 'config', 'scene.yaml')

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.use_ignition = 'true'
    description_loader.gz_control_config_package = 'crane_plus_control'
    description_loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    description_loader.endtip_offset_xyz = '0 0 0.09'
    description_loader.endtip_offset_rpy = '0 -1.57 0'
    description = description_loader.load()

    def robot_state_publisher(context):
        # Gazebo 側の位置制御は P 制御（既定ゲイン 0.1）で、目標の手前で止まりやすい。
        # ロボットの定義（URDF）に position_proportional_gain を差し込んでゲインを上げる
        urdf = description.perform(context)
        gain = LaunchConfiguration('position_gain').perform(context)
        urdf = re.sub(r'<param name="position_proportional_gain">.*?</param>\s*', '', urdf)
        urdf = urdf.replace(
            '<hardware>',
            f'<hardware><param name="position_proportional_gain">{gain}</param>', 1)
        return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                     output='both', parameters=[{'robot_description': urdf}])]

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description', '-name', 'crane_plus',
                   '-z', '1.015', '-allow_renaming', 'true'])

    def spawner(name):
        return ExecuteProcess(cmd=[f'ros2 run controller_manager spawner {name}'],
                              shell=True, output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/tidy/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image'])

    return LaunchDescription([
        DeclareLaunchArgument('scene_file', default_value=default_scene),
        DeclareLaunchArgument('position_gain', default_value='0.5',
                              description='Gazebo の関節位置制御の P ゲイン（既定 0.1 は追従が遅い）'),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=_write_files),
        OpaqueFunction(function=robot_state_publisher),
        spawn,
        spawner('joint_state_broadcaster'),
        spawner('crane_plus_arm_controller'),
        spawner('crane_plus_gripper_controller'),
        bridge,
    ])
