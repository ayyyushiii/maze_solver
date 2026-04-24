from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg   = get_package_share_directory('maze_solver')
    world = os.path.join(pkg, 'worlds', 'maze_world.sdf')
    robot = os.path.join(pkg, 'models', 'robot.sdf')

    return LaunchDescription([

        # ── Gazebo ──────────────────────────────────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'
        ),

        # ── Spawn robot ─────────────────────────────────────────────────────
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'maze_bot',
                '-x', '2.8',   # inside entrance gap, top-left
                '-y',  '-2.8',
                '-z',  '0.1',
                '-Y',  '0',     # yaw=0 → facing east
                '-file', robot
            ],
            output='screen'
        ),

        # ── ROS ↔ Gazebo bridge ─────────────────────────────────────────────
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        # ── BUG2 + SLAM node ────────────────────────────────────────────────
        Node(
            package='maze_solver',
            executable='maze_solver.py',
            output='screen'
        ),

        # ── RViz2 (optional — comment out if not needed) ─────────────────────
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen'
        # ),
    ])