#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import TimerAction # adds delays between starts
import os

def generate_launch_description():

    px4_sitl = ExecuteProcess(
        cmd = [
            'bash', 
            '-c', 
            'cd /home/adwaith/PX4-Autopilot && make px4_sitl gazebo-classic_iris_downward_depth_camera'
        ],
        output = 'screen'
    )

    dds_agent = TimerAction(
        period = 5.0,
        actions = [ExecuteProcess(
                cmd = ['MicroXRCEAgent', 'udp4', '-p', '8888'],
                output = 'screen'
            )
        ]
    )
    detection_node = TimerAction(
        period = 15.0,
        actions = [
            Node(
                package = 'sar_detection',
                executable = 'detection_node',
                output = 'screen'
            )
        ]
    )
    geotag_node = TimerAction(
        period = 15.0,
        actions = [
            Node(
                package = 'sar_geotag',
                executable = 'geotag_node',
                output = 'screen'
            )
        ]
    )
    offboard_control = TimerAction(
        period = 20.0,
        actions = [
            Node(
                package = 'sar_coverage',
                executable = 'offboard_control',
                output = 'screen'
            )
        ]

    )

    return LaunchDescription([
    px4_sitl,
    dds_agent,
    detection_node,
    geotag_node,
    offboard_control,
])

    