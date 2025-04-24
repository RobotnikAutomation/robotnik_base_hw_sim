import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)

    robotnik_base_hw_sim = get_package_share_directory('robotnik_base_hw_sim')
    config_dir = os.path.join(robotnik_base_hw_sim, 'config/robotnik_io_controller')
    config_path = os.path.join(config_dir, robot + '_controllers.yaml')
    print('Using YAML file:', config_path)

    io_node = Node(
        package='robotnik_base_hw_sim',
        executable='robotnik_io_controller_sim_node',
        name='robotnik_io_controller',
        parameters=[config_path],
        output='screen'
    )

    return [io_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=os.environ.get('ROBOT', ''),
            description='Robot model (hummer, rbtheron)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
