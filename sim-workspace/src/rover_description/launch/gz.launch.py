"""Bring up the rover in Gazebo Harmonic with its ros2_control stack.

This project. This is the WORKING baseline: if this launch does not put a
robot on the ground that drives straight, nothing later in this project can be
measured. a later check verifies it numerically.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("rover_description")

    # GZ_SIM_RESOURCE_PATH, or the mesh does not load.
    #
    # xacro leaves `package://rover_description/meshes/...` in the URDF, and
    # gz-sim rewrites it to `model://rover_description/meshes/...` and then
    # looks for it on its OWN resource path -- which does not include the ROS
    # workspace just because the workspace is sourced. Without this, Gazebo
    # prints three red errors ("Unable to find file with URI ...") and puts the
    # robot on the ground anyway, with the lidar housing missing.
    #
    # It has to be the directory ABOVE the package share dir, because the URI
    # already contains the package name as its first component.
    share_root = os.path.dirname(pkg)
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        f"{share_root}:{existing}" if existing else share_root)
    xacro_file = os.path.join(pkg, "urdf", "rover.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()
    world = os.path.join(pkg, "worlds", "flat.sdf")

    # NOTE on the GUI layout: it lives in the world file's <gui> block, NOT in a
    # separate config. An SDF world's <gui> takes precedence over --gui-config,
    # so a config file passed on the command line is silently ignored. Chasing
    # that cost a rebuild: the flag was accepted, the layout never changed.
    gui = LaunchConfiguration("gui")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        # -r runs immediately; -s/-v are set by the gui argument below.
        launch_arguments={"gz_args": [world, " -r ", gui]}.items(),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "rover",
                   # z clears the ground by 1 cm so the robot settles rather
                   # than starting in penetration. A robot spawned inside the
                   # floor is exactly how you get wheels spinning in a corner.
                   "-x", "0", "-y", "0", "-z", "0.01"],
    )

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster"], output="screen")
    ddc = Node(package="controller_manager", executable="spawner",
               arguments=["diff_drive_base_controller"], output="screen")

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/depth@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="-v4",
                              description="'-s -v4' for headless, '-v4' for GUI"),
        gz, rsp, spawn, bridge,
        # Controllers must not spawn until the model exists in the world, or the
        # controller_manager comes up with no hardware and silently loads nothing.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,   on_exit=[ddc])),
    ])
