# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    # gps_wpf_dir = get_package_share_directory(
    #     "nav2_gps_waypoint_follower_demo")
    # rl_params_file = os.path.join(
    #     gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")
    ekf_local_param = os.path.join(
        get_package_share_directory("cr_localization"),
        "config",
        "ekf_local.yaml",
    )
    ekf_global_param = os.path.join(
        get_package_share_directory("cr_localization"),
        "config",
        "ekf_global.yaml",
    )
    navsat_transform_param = os.path.join(
        get_package_share_directory("cr_localization"),
        "config",
        "navsat_transform.yaml",
    )

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                output="screen",
                parameters=[ekf_local_param, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                output="screen",
                parameters=[ekf_global_param, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[navsat_transform_param, {"use_sim_time": False}],
                remappings=[
                    ("imu/data", "imu"),
                    ("gps/fix", "ublox_gps_node/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
        ]
    )