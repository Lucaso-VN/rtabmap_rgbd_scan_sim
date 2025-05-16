# Example:
#
#   Bringup turtlebot3:
#     $ export TURTLEBOT3_MODEL=waffle
#     $ export LDS_MODEL=LDS-01
#     $ ros2 launch turtlebot3_bringup robot.launch.py
#
#   SLAM:
#     $ ros2 launch rtabmap_demos turtlebot3_rgbd_scan.launch.py
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_brightup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'odom_frame_id': 'odom',
          'subscribe_odom': True,
          'odom_topic': '/odometry/filtered',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'scan_topic': '/scan',
          'use_action_for_goal':True,
          'subscribe_odom_info': True,
          'odom_sensor_sync': True,
          # RTAB-Map's parameters should be strings:
          'Odom/Strategy': '0',
          #'Odom/ResetCountdown': '15',
          #'Odom/GuessSmoothingDelay': '0',
          'Reg/Strategy':'1',               # 1=ICP cho lidar
          'Icp/PointToPlane': "true",       # Kích hoạt ICP Point-to-Plane
          'Icp/MaxCorrespondenceDistance': "0.1",           # Tăng số lần lặp
          'Icp/VoxelSize': "0.05",          # Giảm kích thước voxel để chi tiết hơn
          'Reg/Force3DoF':'true',
          'Icp/PointToPlane': 'true',
          'RGBD/ProximityPathMaxNeighbors': "10",  # Khai báo rõ để tránh cảnh báo
          'RGBD/NeighborLinkRefining':'true',
          'RGBD/OptimizeFromGraphEnd': 'false',
          # Loc du lieu
          'RGBD/ProximityMaxDepth': "5.0",  # Bỏ qua điểm sâu >5m
          'RGBD/RangeMax': '5.0',
          'RGBD/RangeMin': '0.2',
          'qos_image': 0,    # Su dung chat luong dich vu (Qos) cho cam bien, 0=sensor_data/1=parameters/2=services/3=default, ep kieu integer
          #
          'RGBD/ProximityPathFiltering': "true",
          'RGBD/LinearUpdate': "0.01",        # Chỉ cập nhật map khi di chuyển đủ xa
          'RGBD/AngularUpdate': '0.01',       # ~0.17 rad = 10°
          'RGBD/ProximityBySpace': 'true',  # ket hop lidar va rgb-d
          'RGBD/OptimizeStrategy': '2',        # g20=1, GTSAM=2
          'RGBD/OptimizeRobust': 'true',        
          'RGBD/OptimizeMaxError': '0',         # should be 0 if RGBD/OptimizeRobust is true
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          # Cau hinh Nav2
          'Grid/RangeMax':'10.0',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/Sensor':'2', # Use both laser scan and camera for obstacle detection in global map
          'Grid/CellSize': "0.1",      # Kích thước mỗi ô (m)
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1.0',  # All points over 1 meter are ignored
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Grid/FromDepth': 'false',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Vis/FeatureType': "2" ,      # Sử dụng ORB (nhanh)
          'Vis/MaxFeatures': "300",    # Giới hạn số feature
          'Vis/MinInliers': "20",        # Tăng ngưỡng inliers
          'Kp/DetectorStrategy': '2', # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB
          'Mem/TimeThr': '600',
          'Mem/STMSize': '30',          # Chi giu lai 30 node gan nhat
          'Mem/SaveDepth16Format': "false",
          'Mem/DepthCompressionFormat': ".png",
          'topic_queue_size': 20,
          'sync_queue_size': 20,
    }

    remappings=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
        ),
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True,
                         'queue_size': 20,
                         'approx_sync_max_interval': 0.01,
                         'use_sim_time':use_sim_time
                        }],
            remappings=remappings),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        #Node(
        #    package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #    parameters=[parameters],
        #    remappings=remappings),
        
        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02,
                         'use_sim_time': use_sim_time}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera/camera_info'),
                        ('cloud', '/camera/cloud')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters, {'use_sim_time': use_sim_time}],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ])