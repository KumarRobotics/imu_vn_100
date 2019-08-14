import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('imu_vn_100'),
        'config')
    params = os.path.join(config_directory, 'default-imu_vn_100-params.yaml')
    imu_vn_100_node = launch_ros.actions.Node(package='imu_vn_100',
                                              node_executable='imu_vn_100_node',
                                              output='both',
                                              parameters=[params])

    return launch.LaunchDescription([imu_vn_100_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=imu_vn_100_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
