import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    
    camera_model_node=Node(
        package = 'coverage_planner',
        name = 'camera_model_node',
        executable = 'camera_model_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"angle_of_view"      :  1.08559479},
            {"image_resolution_x" :  1920},
            {"image_resolution_y" :  1080}
        ]
    )

    coverage_planner_node=Node(
        package = 'coverage_planner',
        name = 'coverage_planner_node',
        executable = 'coverage_planner_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"overlap"        :  0.1},
            {"minimum_height" :  5.0},
            {"maximum_height" : 30.0}
        ]
    )

    photogrammetry_node=Node(
        package = 'coverage_planner',
        name = 'photogrammetry_node',
        executable = 'photogrammetry_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"images_folder" : "./"}
        ]
    )
    
    ld.add_action(camera_model_node)
    ld.add_action(coverage_planner_node)
    ld.add_action(photogrammetry_node)

    return ld
