from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch.substitutions
import launch_ros.substitutions
import launch_ros.descriptions

def generate_launch_description():
  zumo_description = launch_ros.substitutions.FindPackageShare(package='zumo_description').find('zumo_description')
  
  return LaunchDescription([
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          # Set robot_description using command to process xacro
          parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue( 
              launch.substitutions.Command(['xacro ',os.path.join(zumo_description,'urdf/zumo_static.urdf.xacro')]), value_type=str)}]
      )
  ])