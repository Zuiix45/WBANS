from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def load_waypoints(file_path):
    waypoints = open(os.path.normpath(file_path), "r").readlines()
    
    for i in range(len(waypoints)):
        waypoints[i] = waypoints[i].replace("\n", "")
        waypoints[i] = waypoints[i].replace("'", "")
    
    return waypoints

def generate_launch_description():

    xrce_dds_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888'
        ]],
        shell=True
    )

    route_manager_node = Node(
        package='drone',
        executable='route_manager',
        output='screen',
        parameters=[{"waypoints": load_waypoints("./waypoints.txt"), "tolerance_factors": [0.005, 0.005, 0.5]}],
        shell=True
    )

    return LaunchDescription([
        xrce_dds_agent,
        route_manager_node
    ])