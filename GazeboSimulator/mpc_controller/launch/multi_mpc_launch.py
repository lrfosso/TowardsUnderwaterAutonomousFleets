# Importing modules and functions
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml

# Setup of Launch
def multi_launch(context, *args, **kwargs):
    """
    Setting up launch of multiple nodes
    """
    # Getting the file mpc_controller/params/params.yaml
    param_path = os.path.join(
        get_package_share_directory('mpc_controller'),
        'params',
        'params.yaml'
    )

    # Storing dictionary of parameters to a variable
    with open(param_path, 'r') as f:
        config = yaml.safe_load(f)
        param = config['/**']['ros_parameters']

    multi_agent = param['multi_agent']  # Quantity of fleet
    FOV_max = param['FOV_range_deg']    # Hard constraint - Field of View
    launch_list = []                    # Empty list used to store nodes for launch

    # Creating mpc node for every rov in the fleet
    for agent_id in range(multi_agent):
        agent = Node(
            package='mpc_controller',
            namespace='bluerov{}_mpc'.format(agent_id+2),   # Customize node name
            name='bluerov{}'.format(agent_id+2),            # Node shown in list as : /bluerov{}_mpc/bluerov{}
            executable='bluerov_mpc',                       # Node executable from setup.py
            output='screen' if (agent_id+2) == param['debug_rov'] else "log", 
            parameters= [param,
            {
                'main_id': agent_id+2,
                'fleet_quantity': multi_agent,
            }
            ]
        )
        launch_list.append(agent)

    trajectory_node = Node(
       package='mpc_controller',
       executable='setpoint',       # Node executable name from setup.py
       output='log',
       parameters=[param]
    )
    launch_list.append(trajectory_node)
    
    joystick_node = Node(
       package='joy_con',
       executable='joy_con_node',   # Node executable name from CMakeList
       output='log',
    )
    launch_list.append(joystick_node)

    joy_node = Node( # Package that is included with ROS2
       package='joy',
       executable='joy_node',
       output='log',
    )
    launch_list.append(joy_node)

    gui_node = Node( # Graphical User Interface
        package='mpc_controller',
        executable='GUI', # Node executable name from setup.py
        output='log',
        parameters=[
        {
            'fleet_quantity': multi_agent,
            'FOV_max': FOV_max,  
        }
        ]
    )
    launch_list.append(gui_node)

    return launch_list

#Launching nodes
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=multi_launch)
    ])
