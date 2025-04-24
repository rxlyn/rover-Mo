#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  """
  Naming Conventions
  executable = file name + function
  name = file name + node
  ex. 
  executable='commandLine'
  name='commandLineNode'
  """

  
  """
  User Interface Nodes

  User points of connection
  - Terminal
  - GUI
  """
  
  #Command Line Publisher Node
  consoleNode = Node(
      package='mo',                # name of your package
      executable='commandLine',    # name of the node’s executable
      name='commandLineNode',      # optional remapped node name
      output='screen',             # send stdout to screen
  )

  GUINode = Node(
      package='mo',                # name of your package
      executable='ros2GUI',        # name of the node’s executable
      name='ros2GUINode',          # optional remapped node name
      output='screen',             # send stdout to screen
  )

  """
  External Connection Nodes

  Data read and write interface
  - SSH (Paramiko) [SEND | Raspberry Pi]
  - GStreamer [RECIEVE | Raspberry Pi]
  - SensorStream [RECIEVE | Pixel XL]
  """
  
  #SSH Connection Node
  paramikoNode = Node(
      package='mo',                # name of your package
      executable='sshConnect',     # name of the node’s executable
      name='sshConnectNode',       # optional remapped node name
      output='screen',             # send stdout to screen
  )
  
  #Camera Feed Import Node
  GStreamNode = Node(
      package='mo',                # name of your package
      executable='GStream',        # name of the node’s executable
      name='GStreamNode',          # optional remapped node name
      output='screen',             # send stdout to screen
  )

  #Sensors Import Node
  #TODO: Create node

  """
  Internal Process Nodes

  ROS2 Internal Nodes
  - Stereoscopic Mapping of 2 camera feeds
  - Autonomous Driving controls (Collision Avoidence)
  """
  
  #Stereoscopic Depth Mapping Node
  stereoProcessNode = Node(
      package='mo',                # name of your package
      executable='stereoProcess',  # name of the node’s executable
      name='stereoNode',           # optional remapped node name
      output='screen',             # send stdout to screen
  )
  
  #Autonomy Control Node  
  autonomyNode = Node(
      package='mo',                # name of your package
      executable='autonomy',       # name of the node’s executable
      name='autonomyNode',         # optional remapped node name
      output='screen',             # send stdout to screen
  )

  return LaunchDescription([
      consoleNode,
      GUINode,
      paramikoNode,
      GStreamNode,
      stereoProcessNode,
      autonomyNode,
      # you can add more Node() or other launch actions here
  ])
