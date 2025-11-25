from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_path = PathJoinSubstitution(
        [FindPackageShare("robot_ter_bringup"), "worlds", "robot_ter_city.world"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    turtlebot3_model_file = LaunchConfiguration("turtlebot3_model_file")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("gazebo_ros"),
                "/launch/gazebo.launch.py",
            ]
        ),
        launch_arguments={"world": world_path}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("turtlebot3_description"),
                                "urdf",
                                turtlebot3_model_file,
                            ]
                        ),
                    ]
                ),
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robot_ter_bot",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "turtlebot3_model_file",
                default_value="turtlebot3_burger.urdf.xacro",
                description="TurtleBot3 URDF file name from turtlebot3_description/urdf",
            ),
            gazebo,
            robot_state_publisher,
            spawn_entity,
        ]
    )
