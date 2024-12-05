from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    


    pose_conversion_node  = Node(   package='uclv_aruco_detection',
                                    executable='pose_conversion_server_node',
                                    name='pose_conversion_server',
                                    remappings=[
                                        ('/aruco_marker_poses', '/marker_publisher/markers'),     
                                    ],
                                    output='log'
                                )



    return LaunchDescription([
        pose_conversion_node
    ])