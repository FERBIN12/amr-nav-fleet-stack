"""Bring up the Cortex AMR in the warehouse: sim, robot, bridge, TF.

    ros2 launch cortex_amr_description warehouse.launch.py

WHY THE SDF STEP IS HERE AND NOT OPTIONAL. Spawning the raw URDF hands the
physics engine 21 independent rigid bodies, because nothing welds the 18 fixed
joints, and the robot arrives as a pile of parts on the floor. `gz sdf -p`
welds them into 3: chassis, left wheel, right wheel. an earlier module is about
exactly this, so the launch file does it the correct way rather than papering
over it.
"""
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cortex_amr_description")
    xacro_file = os.path.join(pkg, "urdf", "cortex_amr.urdf.xacro")
    bridge_cfg = os.path.join(pkg, "config", "bridge.yaml")

    # xacro -> URDF -> SDF, once, at launch time.
    urdf = subprocess.run(["xacro", xacro_file],
                          capture_output=True, text=True, check=True).stdout
    tmp = tempfile.mkdtemp(prefix="cortex_amr_")
    urdf_path = os.path.join(tmp, "cortex_amr.urdf")
    sdf_path = os.path.join(tmp, "cortex_amr.sdf")
    with open(urdf_path, "w") as fh:
        fh.write(urdf)
    with open(sdf_path, "w") as fh:
        fh.write(subprocess.run(["gz", "sdf", "-p", urdf_path],
                                capture_output=True, text=True,
                                check=True).stdout)

    # model:// resolves against the PARENT of the package dir, so the share
    # root goes on the path. Get this wrong and the meshes silently do not load
    # and the robot renders as scattered primitives, which looks identical to
    # the un-welded failure above and is a different bug.
    share_root = os.path.dirname(pkg)
    os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(filter(None, [
        "/opt/ros/jazzy/share", share_root,
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(filter(None, [
        "/opt/ros/jazzy/lib", os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]))
    # PRIME offload. LIBGL_ALWAYS_SOFTWARE=1 segfaults this machine inside
    # libEGL_mesa; do not set it.
    os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"

    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse",
                              description="warehouse | depot | maze"),

        ExecuteProcess(
            cmd=["gz", "sim", "-v2", "--render-engine", "ogre2",
                 ["/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/",
                  world, ".sdf"]],
            output="screen"),

        Node(package="ros_gz_sim", executable="create",
             arguments=["-world", world, "-file", sdf_path,
                        "-name", "cortex_amr",
                        "-x", "1.2", "-y", "0.0", "-z", "0.18"],
             output="screen"),

        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": bridge_cfg}],
             output="screen"),

        # robot_state_publisher owns the fixed-joint TF the SDF welded away.
        # Gazebo publishes odom -> base_footprint; this publishes everything
        # from base_link down, which is what RViz2 and Nav2 need.
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf, "use_sim_time": True}],
             output="screen"),
    ])
