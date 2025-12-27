import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value = os.path.join(
            get_package_share_directory('bumperbot_mapping'),
            'config',
            'slam_toolbox.yaml',
        ),
    )    
    
    # Creating runtime handles for launch arguments. Launch arguments are strings at launch time, not Python variables.
    # You must convert them into LaunchConfiguration objects to pass into nodes.    
    slam_config = LaunchConfiguration('slam_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launching synchronous SLAM Toolbox node.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',    # Processes scans one-by-one
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
        ]
    )

    # To store/save the map created using the toolbox (Launching a service-based map saver).
    nav2_map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[
            {'save_map_timeout': 3.0},
            {'use_sim_time': use_sim_time},
            {'free_thresh_default': 0.196},       # Defines how probabilities become pixels (< 0.196 are free cells)
            {'occupied_threshold_default': 0.65}, # Defines how probabilities become pixels (> 0.65 are occupied cells)
        ],
    )

    # Since most of the nav2 nodes are developed as lifecycle nodes, we need to bring them up 
    # through a lifecycle manager node. In ROS2-Humble, slam_toolbox is not managed as a lifecycle node.
    lifecycle_nodes = ['map_saver_server']
    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro != 'humble':
        lifecycle_nodes.append('slam_toolbox')
    
    # Automatically transitioning nodes through: unconfigured → inactive → active
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,           # Consistent simulation time
        slam_config_arg,            # SLAM Toolbox configuration file
        slam_toolbox,               # SLAM Toolbox node
        nav2_map_saver,             # Manual map saving
        nav2_lifecycle_manager,     # Correct lifecycle transitions
    ])


'''
Chronology of events in Launch File:
------------------------------------
    Launch file parsed
    ↓
    Launch arguments registered
    ↓
    Nodes described (not running)
    ↓
    Arguments resolved
    ↓
    slam_toolbox starts (ACTIVE)
    ↓
    map_saver_server starts (UNCONFIGURED)
    ↓
    lifecycle_manager starts
    ↓
    map_saver_server → ACTIVE
    ↓
    /clock sync established
    ↓
    SLAM running
    ↓
    Map saved on request
------------------------------------
'''