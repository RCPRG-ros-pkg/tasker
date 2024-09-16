from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    harmonizer_name_arg = DeclareLaunchArgument(
        'harmonizer_name', default_value='THA'
    )
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode', default_value='gazebo'
    )
    kb_places_xml_arg = DeclareLaunchArgument(
        'kb_places_xml', default_value=os.path.join(
            get_package_share_directory('tasker'), 
            'maps', '012_places', 'places.xml'
        )
    )

    # Define nodes
    task_harmonizer_node = Node(
        package='tasker',
        executable='task_harmonizer',
        name=LaunchConfiguration('harmonizer_name'),
        output='screen',
        parameters=[
            {'harmonizer_name': LaunchConfiguration('harmonizer_name')},
            {'sim_mode': LaunchConfiguration('sim_mode')}
        ]
    )

    dictionary_service_node = Node(
        package='pl_nouns',
        executable='dictionary_service',
        name='dictionary_service',
        output='screen',
        parameters=[{'kb_places_xml': LaunchConfiguration('kb_places_xml')}]
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription()

    # Add the actions (arguments and nodes) to LaunchDescription
    ld.add_action(harmonizer_name_arg)
    ld.add_action(sim_mode_arg)
    ld.add_action(kb_places_xml_arg)
    ld.add_action(task_harmonizer_node)
    ld.add_action(dictionary_service_node)

    return ld


# <launch>
# 	<arg name='harmonizer_name' default='THA'/>
# 	<arg name='sim_mode' default='gazebo'/>
# 	<arg name='kb_places_xml' default=' $(find tiago_sim_integration)/maps/012_places/places.xml'/>
# 	<param name='harmonizer_name' value='$(arg harmonizer_name)'/>
# 	<param name='sim_mode' value='$(arg sim_mode)'/>
# 	<param name='kb_places_xml' value='$(arg kb_places_xml)'/>
# 	<node name='$(arg harmonizer_name)' pkg='tasker' type='task_harmonizer' output='screen'>
# 	</node>
# 	<node name='dictionary_service' pkg='pl_nouns' type='dictionary_service' output='screen'>
# 	</node>
# </launch>