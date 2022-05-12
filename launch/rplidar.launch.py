from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_package",
            default_value="robot_common",
            description="Robot package package with ldar data included",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="rplidar.yaml",
            description="The parameter file name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='r3/',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    robot_package = LaunchConfiguration("robot_package")
    config_file = LaunchConfiguration("config_file")
    prefix = LaunchConfiguration("prefix")
 
    # RpLidar Config Path
    rplidar_param_file = PathJoinSubstitution(
        [
            FindPackageShare(robot_package),
            "config",
            config_file,
        ]
    )

    #Replace Config file wiht prefix

    namespaced_param_config_file = ReplaceString(
        source_file=rplidar_param_file,
        replacements={'prefix/': prefix})

    #RpLidar Node 
    lidar_node=Node(
        name='rplidar_node',
        namespace= prefix,
        package = 'rplidar_ros',
        executable = 'rplidar_composition',
        parameters = [namespaced_param_config_file]
    )


    nodes = [
        lidar_node,
    ]

    return LaunchDescription(declared_arguments + nodes)