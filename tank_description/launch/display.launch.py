#!/usr/bin/env python3
import imp
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import xacro

def generate_launch_description():

    ### How to get path of a file from "install" directory ###
    #
    ## If our custom package [package_name] consists of a folder [folder_name].
    ## And inside that folder, there is a file name [file_name.extension].
    ## This is how to get full path to the file (in a form of string)

    path_to_package = get_package_share_directory('tank_description')
    sub_folder = 'config'
    file_name = 'tank_kinematics.rviz'
    file_path = os.path.join(path_to_package,sub_folder,file_name)  # join path  
    
    ## You must build the custom package before. Otherwise, these codes will not find the file
    # in "install" directory

    ### How to run rviz2 ###
    #
    ## With -d rviz_file_path, you can load the previous setting of rviz
    #
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz',
       arguments=['-d', file_path],
       output='screen')
    
    ### How to read URDF file ###
    #
    # Assume that you have the correct path [robot_description_path]
    # with open(robot_description_path, 'r') as infp:
    #     robot_description = infp.read()

    robot_desc_path = os.path.join(get_package_share_directory(
                                    'tank_description'), 
                                    'robot',
                                    'tank.urdf')

    robot_description = xacro.process_file(robot_desc_path).toxml() #read URDF file from xacro

    # create node: robot_state_publisher 
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[
                                    # {'use_sim_time': 'false'},
                                    {'robot_description': robot_description}
    ])

    # joint_state_publisher = Node(package='joint_state_publisher',
    #                                 executable='joint_state_publisher',
    #                                 name='joint_state_publisher'
    # )
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui'
    )
                                    
   
    # You must create an object called "LaunchDescription"
    launch_description = LaunchDescription()
      # and add actions one by one
    launch_description.add_action(rviz)     #and add actions rivz
    launch_description.add_action(robot_state_publisher)    #and add actions robot_state_publisher
    # launch_description.add_action(joint_state_publisher)
    # launch_description.add_action(joint_state_publisher_gui)
    
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    