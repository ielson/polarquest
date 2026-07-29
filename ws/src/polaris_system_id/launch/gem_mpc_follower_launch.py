"""ROS2 Python launch file for GEM MPC follower.

GAZEBO / CONTROLLER INFRASTRUCTURE IS DEFERRED TO PHASE 2.
This launch file starts only the polaris_system_id core nodes.
The following are NOT included here and must be added in Phase 2:
  - Gazebo bringup (gz sim + ros_gz_bridge)
  - gem_gazebo bringup
  - car_like_v_angle_controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # MPC parameters
    mode = LaunchConfiguration('mode', default='speed_steer')
    use_curv_speed = LaunchConfiguration('use_curv_speed', default='true')
    curv_gain = LaunchConfiguration('curv_gain', default='6.0')
    spd_min = LaunchConfiguration('spd_min', default='0.5')
    spd_max = LaunchConfiguration('spd_max', default='8.0')
    spd_slew = LaunchConfiguration('spd_slew', default='0.6')

    set_env_vars = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
    ]

    odom_to_tf_node = Node(
        package='polaris_system_id',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{
            'odom_topic': '/gem/base_footprint/odom',
            'parent_frame': '',
            'child_frame': '',
        }],
    )

    path_from_csv_node = Node(
        package='polaris_system_id',
        executable='publish_from_csv',
        name='path_from_csv',
        output='screen',
        parameters=[{
            'frame_id': 'world',
            'topic_path': '/path_xy',
            'repeat': False,
            'rate_hz': 1.0,
        }],
    )

    mpc_node = Node(
        package='polaris_system_id',
        executable='mpc',
        name='mpc',
        output='screen',
        parameters=[{
            'mode': mode,
            'use_curv_speed': use_curv_speed,
            'curv_gain': curv_gain,
            'spd_min': spd_min,
            'spd_max': spd_max,
            'spd_slew': spd_slew,
        }],
    )

    return LaunchDescription([
        # Env vars for nicer logging
        *set_env_vars,

        # Odometry to TF
        odom_to_tf_node,

        # Path publisher
        path_from_csv_node,

        # MPC controller
        mpc_node,

        # TODO(Phase 2): Gazebo bringup (gz sim + ros_gz_bridge for gem)
        # TODO(Phase 2): car_like_v_angle_controller
    ])
