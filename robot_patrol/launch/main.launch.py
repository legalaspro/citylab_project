import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

     # Get Package Description and Directory #
    package_description = "robot_patrol"
    package_directory = get_package_share_directory(package_description)

    # Load RViz Configuration File #
    rviz_config_file = "config.rviz"
    rviz_config_path = os.path.join(package_directory, "rviz", rviz_config_file)
    print("RViz Config Loaded !")


    #Nodes
    # Service launching node
    service_node = Node(
        package="robot_patrol",
        executable="direction_service_executable",
        output='screen',
        arguments=[],
        emulate_tty=True
    )

    # Patrol launching node
    patrol_node = Node(
        package="robot_patrol",
        executable="patrol_with_service_executable",
        arguments=[],
        emulate_tty=True,
        output='screen',
    )

    # RViz2 Launch Configuration (RViz) #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
    )

    return launch.LaunchDescription([
        service_node,
        patrol_node,
        rviz_node
    ])