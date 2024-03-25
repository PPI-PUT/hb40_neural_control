# Copyright 2024 Maciej Krupka
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('hb40_neural_controller_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('hb40_neural_controller'), 'config', 'hb40_neural_controller.param.yaml']
        ).perform(context)

    hb40_neural_controller_node = Node(
        package='hb40_neural_controller',
        executable='hb40_neural_controller_node_exe',
        name='hb40_neural_controller_node',
        parameters=[
            param_path
        ],
        remappings=[
            ("~/input/robot_state", LaunchConfiguration("input_robot_state")),
            ("~/input/bridge_data", LaunchConfiguration("input_bridge_state")),
            ("~/input/cmd_vel", LaunchConfiguration("input_cmd_vel")),
            ("~/output/joint_command", LaunchConfiguration("output_joint_command")),
            ("~/input/system_cmd", LaunchConfiguration("input_system_cmd"))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
    )

    return [
        hb40_neural_controller_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('hb40_neural_controller_param_file', '')
    add_launch_arg("input_robot_state", "/hb40/robot_state")
    add_launch_arg("input_bridge_state", "/hb40/bridge_data")
    add_launch_arg("input_cmd_vel", "/hb40/velocity_command")
    add_launch_arg("output_joint_command", "/hb40/joint_commandHighPrio")
    add_launch_arg("input_system_cmd", "/hb40/control_command")
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
