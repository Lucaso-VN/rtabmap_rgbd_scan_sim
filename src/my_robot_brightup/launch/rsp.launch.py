import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_brightup'),
        'description',
        'robot.urdf.xacro'
    )
    
    doc = xacro.process_file(urdf_file)
    robot_des = doc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_des}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])
