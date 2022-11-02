#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys

def generate_launch_description():

    ### How to launch another launch file ###
    # You must specify the package, folder, and the name
    display = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tank_description'),   #use package tank_description
                'launch',   #file to run are in this folder
                'display.launch.py' #want to launch file display.launch.py
            ])
        ])
    )
    # create node: kinematics_server
    kinematics_server = Node(
        package='tank_kinematics',  #use package tank_kinematics
        executable='kinematics_server.py',  #want to run file kinematics_server.py
        name='tank_kinematics'   #name
        # output='screen'
        )
        
    # Launch Description
    launch_description = LaunchDescription()
    # and add actions one by one
    launch_description.add_action(display)  #and add actions display
    launch_description.add_action(kinematics_server)    #and add actions kinematics_server

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    