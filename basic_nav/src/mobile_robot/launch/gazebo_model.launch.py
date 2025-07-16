import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro
#Start Gazebo Simulation, bridge, additional node robotstatepublisher
def generate_launch_description():
#How nodes are being started
    #this name has to match the robot name in the xacro file
    robotXacroName = 'differential_drive_robot'

    #name of the package and the same time is t name of the folder used to define paths
    namePackage = 'mobile_robot'

    #relative path to hte xacro file defining the model
    modelFileRelativePath = 'model/robot.xacro'
    
    worldFileRelativePath = 'worlds/basic.sdf'

    #relative path to gazebo world file


    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)

    #own world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    #robotDescription = xacro.process_file(pathModelFile).toxml()
    robotDescription = xacro.process(pathModelFile)
    
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    #this is the launch description

    #empty world model
    #gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown':'true'}.items())

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': f'-r -v4  {pathWorldFile}', 
            'on_exit_shutdown': 'true'
        }.items()
    )

    #Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic','robot_description'
            ],
        output = 'screen',
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robotDescription,
                     'use_sim_time':True}]               
    )

#This is very importatnt soo we can control the robot from ros2
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package = 'ros_gz_bridge',
        executable= 'parameter_bridge',
        arguments = [
            '--ros-args',
            '-p',
            f'config_file:= {bridge_params}',
        ],
        output='screen',
    )

    #here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    #add Gazebo Launch
    launchDescriptionObject.add_action(gazeboLaunch)

    #add both nodes
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject

