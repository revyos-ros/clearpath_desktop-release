# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def local_hostname():
    """
    Get the local hostname as a valid ROS namespace

    Replaces dashes with underscores, adds an 'a' prefix if the first character
    is a number (an underscore would result in it being a hidden topic, so use a letter)
    """
    try:
        with open('/etc/hostname', 'r') as hostname_file:
            hostname = hostname_file.readline()

        hostname = hostname.strip().lower().replace('-', '_')
        if hostname[0].isdigit():
            hostname = f'a{hostname}'
    except Exception as err:
        print(f'Failed to read hostname: {err}')
        hostname = 'offboard_computer'
    return hostname


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    offboard_namespace = LaunchConfiguration('offboard_namespace')
    parameters = LaunchConfiguration('parameters')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
    )

    arg_offboard_namespace = DeclareLaunchArgument(
        'offboard_namespace',
        default_value=local_hostname(),
    )

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_offboard_sensors'),
          'config',
          'velodyne_lidar.yaml'
        ]),
    )

    velodyne_pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        namespace=(namespace, '/', offboard_namespace),
        output='screen',
        parameters=[parameters],
        remappings=[
          ('/diagnostics', 'diagnostics'),
          ('velodyne_packets', (namespace, '/velodyne_packets')),
          ('velodyne_points', 'points'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_offboard_namespace)
    ld.add_action(arg_parameters)
    ld.add_action(velodyne_pointcloud_node)
    return ld
