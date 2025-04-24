#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  
  #Camera Feed Import Node
  autonomyNode = Node(
      package='mo',                # name of your package
      executable='GStreamImport',  # name of the node’s executable
      name='GStreamNode',          # optional remapped node name
      output='screen',             # send stdout to screen
  )

  #Stereoscopic depth mapping Node
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
        autonomyNode,
        # you can add more Node() or other launch actions here
    ])
