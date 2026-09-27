"""Launch the tram backup odometry node.

    ros2 launch tram_backup_odometry backup_odometry.launch.py \
        [params_file:=<yaml>] [output_frame:=map|utm|start|enu] [vehicle_id:=30618|30639] \
        [use_sim_time:=false] [maps_dir:=<dir>] [log_level:=info]

``params_file`` defaults to the installed config/params.yaml.  ``output_frame`` and
``maps_dir`` override the file only when given (non-empty), so an edited params file is
not silently shadowed by launch defaults.  ``vehicle_id`` defaults to 30618 (the hidden
check is run on tram 30618); pass ``vehicle_id:=30639`` for the other tram or
``vehicle_id:=''`` to take the value from the params file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'tram_backup_odometry'


def _node(context):
    cfg = context.launch_configurations
    overrides = {'use_sim_time': cfg['use_sim_time'].strip().lower() in ('1', 'true', 'yes', 'on')}
    for key in ('output_frame', 'vehicle_id', 'maps_dir'):
        val = cfg[key].strip()
        if val:
            overrides[key] = ParameterValue(val, value_type=str)
    return [Node(
        package=PKG,
        executable='backup_odometry_node',
        name='backup_odometry',
        output='screen',
        emulate_tty=True,
        parameters=[cfg['params_file'], overrides],
        arguments=['--ros-args', '--log-level', cfg['log_level']],
    )]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'params.yaml'),
                              description='ROS 2 parameter file (backup_odometry: ros__parameters: ...)'),
        DeclareLaunchArgument('output_frame', default_value='',
                              description='map | utm | start | enu (empty = value from params_file)'),
        DeclareLaunchArgument('vehicle_id', default_value='30618',
                              description='30618 | 30639 selects the calibrated wheel scale (empty = params_file)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='results do not depend on it (vehicle header clock is used)'),
        DeclareLaunchArgument('maps_dir', default_value='',
                              description='directory with t2s.json / s2t.json (empty = installed maps)'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_node),
    ])
