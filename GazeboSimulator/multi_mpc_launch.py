from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

#Setup of Launch
def multi_launch(context, *args, **kwargs):
    multi_agent = int(LaunchConfiguration('multi_agent').perform(context))
    #Creating a node for each ROV
    launch_agents = []
    for agent_id in range(multi_agent):
        agent = Node(
            package='mpc_controller',
            namespace='bluerov{}_mpc'.format(agent_id+2),
            name='bluerov{}'.format(agent_id+2),
            executable='bluerov_mpc',
            #output='screen',
            #prefix=['gnome-terminal --command gdb'],
            parameters= [
            {
                'main_id': str(agent_id+2),
                'fleet_quantity': str(multi_agent),
            }
            ]
        )
        launch_agents.append(agent)

    trajectory_node = Node(
       package='mpc_controller',
       executable='setpoint',
       output='screen'
    )
    launch_agents.append(trajectory_node)

    return launch_agents

#Launch
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('multi_agent', default_value='1'),
        OpaqueFunction(function=multi_launch)
    ])