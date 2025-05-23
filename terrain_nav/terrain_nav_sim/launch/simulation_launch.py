import os
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution


def generate_launch_description():
    pkg_terrain_nav_sim = get_package_share_directory(
        "terrain_nav_sim"
    )
    available_world_names = [
        f[:-4]
        for f in os.listdir(os.path.join(pkg_terrain_nav_sim, "worlds"))
        if f.endswith(".sdf")
    ]

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "world_name",
            description="Name of the world to simulate"
            + '(see terrain_nav\' "worlds" directory).',
            default_value=available_world_names[0],
            choices=available_world_names,
        ),
        DeclareLaunchArgument(
            "start_gazebo_gui",
            description="Start Gazebo GUI",
            default_value="True",
            choices=["True", "False"],
        ),
    ]
    world_name = LaunchConfiguration("world_name")
    world_path = PathJoinSubstitution(
        [
            pkg_terrain_nav_sim,
            "worlds",
            PythonExpression(['"', world_name, '" + ".sdf"']),
        ]
    )
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")

    pkg_scout_description = get_package_share_directory("scout_description")
    
    set_ign_resource_path = SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=os.path.dirname(pkg_scout_description))
    set_gz_model_path = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=os.path.dirname(pkg_scout_description))

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
              PathJoinSubstitution([pkg_scout_description, "urdf", "scout_mini", "scout_mini.xacro"]),
            " name:=robot",
            " prefix:='robot'",
            " is_sim:=true",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "publish_frequency": 100.0,
                "robot_description": robot_description,
            }
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_path,  # which world to load
                PythonExpression(
                    ['"" if ', start_gazebo_gui, ' else " -s"']
                ),  # whether to start gui
            ]
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            # The robot's name in simulation is always "robot", regardless of which model is chosen
            # This facilitates easier topic bridging.
            "robot",
            "-z",
            "1",
        ],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Bridge between ROS and Gazebo
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": PathJoinSubstitution([pkg_terrain_nav_sim, "config", "ros_gazebo_bridge.yaml"]),}],
        output="screen",
    )
    robot_controllers = PathJoinSubstitution([FindPackageShare("scout_description"),"config","control.yaml",])

    # Controller Manager Node
    # controller_manager = Node(package="controller_manager",
    #                           executable="ros2_control_node",
    #                           parameters=[robot_controllers],
    #                           output="both",)

   # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(package="controller_manager", executable="spawner", arguments=["scout_state_broadcaster"], output="both",)

    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["scout_base_controller"], output="both", )

    relay_odom = Node( name="relay_odom", package="topic_tools", executable="relay",
    parameters=[{"input_topic": "/scout_base_controller/odom", "output_topic": "/odom",}], output="both",)

    relay_cmd_vel = Node(name="relay_cmd_vel", package="topic_tools", executable="relay",
        parameters=[{"input_topic": "/cmd_vel", "output_topic": "/scout_base_controller/cmd_vel",}],output="both",)

    return LaunchDescription([
        set_ign_resource_path,
        set_gz_model_path,
        *launch_args,
        gz_sim,
        spawn_robot,
        bridge,
        robot_state_publisher,
        # controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        relay_odom,
        relay_cmd_vel
    ])