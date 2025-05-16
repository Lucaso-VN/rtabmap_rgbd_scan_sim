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
          #'odom_frame_id': 'odom',
          'publish_tf_odom': 'false',  # use odom from robot_localization
          #'subscribe_depth': True,
          #'subscribe_stereo': True,
          'subscribe_odom': True,
          'odom_topic': '/odometry/filtered',
          'imu_topic': '/demo/imu/data',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'scan_topic': '/scan',
          'use_action_for_goal':True,
          'subscribe_odom_info': True,
          'odom_sensor_sync': True,
          'map_negative_poses_ignored': True,
          #'wait_imu_to_init': 'true',
          'wait_for_transform': 0.5,    #delay to avoid some TF warning
          'visual_odometry': 'true',
          # RTAB-Map's parameters should be strings:
          'Odom/Strategy': '0',
          'publish_null_when_lost': False,  # không publish giá trị null, giữ nguyên giá trị odometry cuối cùng
          'Odom/ResetCountdown': '1',   # Reset ngay lập tức khi phát hiện loop closure hay đk reset
          'Odom/FillInfoData': 'true',  # thu thập thông tin (key features, độ tin cậy ước tính) đưa vào dữ liệu odometry
          'OdomF2M/MaxSize': '800',    # giới hạn số lượng keyframe tối đa trong bộ nhớ để xây dựng bản đồ  odometry
          'Odom/GuessSmoothingDelay': '0',
          'Reg/Strategy':'1',               # 1=ICP cho lidar
          'Icp/PointToPlane': "true",       # Kích hoạt ICP Point-to-Plane
          'Icp/MaxCorrespondenceDistance': "0.1",           # Tăng khoảng cách bắt điểm chi tiết
          'Icp/VoxelSize': "0.05",          # Giảm kích thước voxel để chi tiết hơn
          'Reg/Force3DoF':'true',
          'Icp/PointToPlane': 'true',
          'Icp/CorrespondenceRatio': '0.2',         # minimum scan overlap to accept loop closure
          'RGBD/ProximityPathMaxNeighbors': "10",  # Khai báo rõ để tránh cảnh báo
          'RGBD/NeighborLinkRefining':'true',       # Do odometry correction with consecutive laser scans
          'RGBD/OptimizeFromGraphEnd': 'false',
          # Loc du lieu
          'RGBD/ProximityMaxDepth': "5.0",  # Bỏ qua điểm sâu >5m
          'RGBD/RangeMax': '5.0',
          'RGBD/RangeMin': '0.2',
          #'qos_image': 0,    # Su dung chat luong dich vu (Qos) cho cam bien, 0=sensor_data/1=parameters/2=services/3=default, ep kieu integer
          #
          'RGBD/ProximityPathFiltering': "true",
          'RGBD/LinearUpdate': "0.01",        # Chỉ cập nhật map khi di chuyển đủ xa
          'RGBD/AngularUpdate': '0.01',       # ~0.17 rad = 10°
          'RGBD/ProximityBySpace': 'true',  # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime': 'false',     # Local loop closure detection with locations in STM
          'RGBD/OptimizeMaxError': '4',         # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'RGBD/LocalRadius': '5',          # limit length of proximity detections
          'RGBD/StartAtOrigin': 'true',         # starting with the first node
          #'RGBD/LoopClosureFeatures': '0',      # so luong features de chap nhan loop closure
          #'RGBD/OptimizeStrategy': '2',        # g20=1, GTSAM=2
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          # Cau hinh Nav2
          'Grid/RangeMax':'0',               # o=inf
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/Sensor':'2', # Use both laser scan and camera for obstacle detection in global map
          'Grid/CellSize': "0.05",      # Kích thước mỗi ô (m)
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1.0',  # All points over 1 meter are ignored
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Grid/FromDepth': 'false',        # Create 2D occupancy grid from laser scan
          #'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Vis/UseIMU': "true",        # giảm drift yaw
          'Vis/FeatureType': "2" ,      # Sử dụng ORB (nhanh)
          'Vis/MaxFeatures': "800",    # Giới hạn số feature
          'Vis/MinInliers': "8",        # Tăng ngưỡng inliers
          'Vis/InlierDistance': '0.1',      # so sanh point cloud cua frame hien tai va fram trc, khoang cach point cloud 2 frame > nguong thi loai bo
          'Kp/DetectorStrategy': '2', # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB
          'Kp/MaxDepth': '4.0',             # do sau toi da cua depth cam
          'Kp/MaxFeatures': '800',
          'Kp/MinFeatures': '20',
          'Mem/TimeThr': '700',
          'Mem/STMSize': '30',          # Chi giu lai 30 node gan nhat
          'Mem/NotLinkedNodesKept': 'false',
          'Mem/RehearsalSimilarity': '0.4',
          'Mem/SaveDepth16Format': "false",
          'Mem/DepthCompressionFormat': ".png",
          #'Optimizer/Slam2D': 'true',
          'topic_queue_size': 30,
          'sync_queue_size': 30,
          'Rtabmap/MaxRepublished': '5',
          'Rtabmap/DetectionRate': '3',
          'subscribe_initial_pose': True,
          'initial_pose_topic': 'initialpose'
    }

    remappings=[
          ('odom', '/odometry/filtered'),
          ('scan', '/scan'),
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
                         'rgbd_sync': False,
                         'queue_size': 30,
                         'approx_sync_max_interval': 0.05,
                         'use_sim_time':use_sim_time
                        }],
            remappings=remappings),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings + [  # Thêm remapping cho initialpose
                            ('/initialpose', '/rtabmap/initialpose')],
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'false',
               'Mem/InitWMWithAllNodes':'true'}],
            remappings=remappings + [  # Thêm remapping cho initialpose
                            ('/initialpose', '/rtabmap/initialpose')],
            ),

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
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud', '/camera/cloud')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters, {'use_sim_time': use_sim_time}],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]
            ),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='chassis_to_camera_static_transform',
        #    output='screen',
        #    arguments=[
        #        '0.305',  # x
        #        '0',      # y
        #        '0.08',   # z
        #        '0',      # roll
        #        '0',      # pitch
        #        '0',      # yaw
        #        'chassis',      # parent frame
        #        'camera_link'   # child frame
        #    ]
        #)
    ])