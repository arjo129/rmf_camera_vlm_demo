from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import launch.actions


def generate_launch_description():
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='panda_world'
    )
    pkg_ros_gz_sim = get_package_share_directory('spawn_camera_gz')
    cmd_str= [
              "gz", "service", "-s",
              f"/world/sim_world/create", "--reqtype", "gz.msgs.EntityFactory",
              "--req",
              f"'sdf_filename: \"{pkg_ros_gz_sim}/models/camera.sdf\"'",
              "--reptype", "gz.msgs.Boolean"]

    return LaunchDescription([
        ExecuteProcess(
           cmd=cmd_str,#["gz", "topic", "--list"],
          shell=True,
          output='both'
        ),
    ])