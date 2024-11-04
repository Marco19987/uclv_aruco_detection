from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    config_file = 'object_transformation.yaml'
    object_name = 'resin_block'


    pose_conversion_node  = Node(   package='uclv_aruco_detection',
                                    executable='pose_conversion_node',
                                    name='pose_conversion',
                                    parameters=[{'config_file': os.path.join(get_package_share_directory('uclv_aruco_detection'), 'config', config_file)
                                                ,'object_name': object_name}],
                                    remappings=[
                                        ('/aruco_marker_poses', '/marker_publisher/markers'),     
                                    ],
                                    output='log'
                                )
    multiple_poses_elaboration_node  = Node(   package='uclv_aruco_detection',
                                    executable='multiple_poses_elaboration_node',
                                    name='multiple_poses_elaboration_node',
                                    parameters=[{'config_file': os.path.join(get_package_share_directory('uclv_aruco_detection'), 'config', config_file)
                                                ,'object_name': object_name}],
                                    output='screen'
                                )


    return LaunchDescription([
        pose_conversion_node,
        multiple_poses_elaboration_node
    ])