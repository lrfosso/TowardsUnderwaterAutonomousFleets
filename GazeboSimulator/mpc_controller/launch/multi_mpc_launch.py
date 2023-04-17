from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml

#Setup of Launch
def multi_launch(context, *args, **kwargs):
    #multi_agent = int(LaunchConfiguration('multi_agent').perform(context))

    #Getting the file mpc_controller/params/params.yaml
    param_path = os.path.join(
    get_package_share_directory('mpc_controller'),
    'params',
    'params.yaml'
    )
    with open(param_path, 'r') as f:
        config = yaml.safe_load(f)
        param = config['/**']['ros_parameters']

    multi_agent = param['multi_agent']

    #Creating a node for each ROV
    launch_agents = []
    for agent_id in range(multi_agent):
        agent = Node(
            package='mpc_controller',
            namespace='bluerov{}_mpc'.format(agent_id+2),
            name='bluerov{}'.format(agent_id+2),
            executable='bluerov_mpc',
            output='screen' if (agent_id+2) == param['debug_rov'] else "log",
            parameters= [param,
            {
                'main_id': agent_id+2,
                'fleet_quantity': multi_agent,
            }
            ]
        )
        launch_agents.append(agent)

    trajectory_node = Node(
       package='mpc_controller',
       executable='setpoint',
       output='log',
       parameters=[param]
    )
    launch_agents.append(trajectory_node)

    gui_node = Node(
        package='mpc_controller',
        executable='GUI',
        output='log',
        parameters=[
        {
            'fleet_quantity': multi_agent    
        }
        ]
    )
    launch_agents.append(gui_node)

    return launch_agents

#Launch
def generate_launch_description():
    return LaunchDescription([
        #DeclareLaunchArgument('multi_agent', default_value='1'),
        OpaqueFunction(function=multi_launch)
    ])