import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_brightup')
    
    # Tham số cấu hình
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Tham số launch
        DeclareLaunchArgument(
            'map_file',
            default_value=PathJoinSubstitution([os.path.join('model_ws', 'my_map_save.yaml')]),
            description='Path to existing map'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        # Node RTAB-Map cho localization và point cloud mapping
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'use_sim_time': use_sim_time,
                
                # Cấu hình SLAM
                'Reg/Strategy': '1',  # ICP
                'RGBD/Enabled': 'true',
                'Grid/3D': 'true',
                'Grid/FromDepth': 'true',
                'Grid/RayTracing': 'true',
                'Grid/CellSize': '0.05',
                
                # Cấu hình cảm biến
                'subscribe_rgbd': 'true',
                'subscribe_scan': 'true',
                'scan_cloud_max_points': '0',  # Unlimited
                
                # Cấu hình lưu dữ liệu
                'Database/Path': PathJoinSubstitution([pkg_share, 'database', 'my_map', 'rtabmap.db']),
                'Mem/IncrementalMemory': 'false',
                'Mem/STMSize': '30',
            }],
            remappings=[
                ('scan', '/scan'),
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
            ],
            arguments=['-d']  # Xóa database cũ khi khởi động
        ),

        # Node xử lý RGB-D
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{'approx_sync': False, 'qos': 1}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
            ]
        ),

        # Node chuyển đổi sang point cloud
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb',
            output='screen',
            parameters=[{
                'decimation': 2,
                'voxel_size': 0.01,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('cloud', '/combined_cloud')
            ]
        ),

        # Node Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_share,
                    'launch', 'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    pkg_share, 'config', 'bot_rgbd_scan_nav2_params.yaml'
                ])
            }.items()
        ),

        # Lưu point cloud khi kết thúc
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/rtabmap/export_map',
                 'rtabmap_msgs/srv/ExportMap',
                 '{map_file: ' + PathJoinSubstitution([pkg_share, 'database', 'my_map', 'final_cloud.pcd']) + '}'],
            shell=True,
            output='screen',
            on_exit=True
        )
    ])