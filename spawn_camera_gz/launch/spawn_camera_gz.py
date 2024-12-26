from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import OpaqueFunction
import launch.actions


def launch_setup(context):
    #config = LaunchConfiguration('world_name').perform(context) # Here you'll get the runtime config value

    world_name = launch.substitutions.LaunchConfiguration('world_name')
    x = launch.substitutions.LaunchConfiguration('x').perform(context)
    y = launch.substitutions.LaunchConfiguration('y').perform(context)
    z = launch.substitutions.LaunchConfiguration('z').perform(context)

    pkg_ros_gz_sim = get_package_share_directory('spawn_camera_gz')
    cmd_str= [
              "gz", "service", "-s",
              f"/world/{world_name.perform(context)}/create", "--reqtype", "gz.msgs.EntityFactory",
              "--req",
              f"'sdf_filename: \"{pkg_ros_gz_sim}/models/semantic_camera.sdf\", pose: {{position: {{x: {x}, y: {y}, z: {z} }}}}'",
              "--reptype", "gz.msgs.Boolean"]

    return [ExecuteProcess(
          cmd=cmd_str,
          shell=True,
          output='both'
        )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='sim_world'
        ),
        DeclareLaunchArgument(
            'x',
            default_value="20.42"
        ),
        DeclareLaunchArgument(
            'y',
            default_value="-5.31"
        ),
        DeclareLaunchArgument(
            'z',
            default_value="0.8"
        ),
        DeclareLaunchArgument(
            'qx',
            default_value="0"
        ),
        DeclareLaunchArgument(
            'qy',
            default_value="0"
        ),
        DeclareLaunchArgument(
            'qz',
            default_value="0"
        ),
        DeclareLaunchArgument(
            'qw',
            default_value="1"
        ),
        OpaqueFunction(function = launch_setup),
    ])