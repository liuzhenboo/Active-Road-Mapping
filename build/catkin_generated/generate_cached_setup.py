# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/move_base;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/rotate_recovery;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/global_planner;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/navfn;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/move_slow_and_clear;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/dwa_local_planner;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/clear_costmap_recovery;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/carrot_planner;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/base_local_planner;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/nav_core;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/costmap_2d;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/voxel_grid;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_tools;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_sensor_msgs;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/test_tf2;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_kdl;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/fake_localization;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_geometry_msgs;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_ros;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_py;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_eigen;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_bullet;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/map_server;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/tf2_msgs;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/robot_pose_ekf;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/navigation;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/move_base_msgs;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/map_msgs;/media/lzb/Windows-SSD/ubuntu18/Navigation_WS/devel_isolated/geometry2;/home/lzb/Realsense/ros_ws/devel;/home/lzb/ros_projects/ZED_ROS_Wapper/devel;/opt/ros/melodic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/lzb/ros_ws/src/Active-Road-Mapping/build/devel/env.sh')

output_filename = '/home/lzb/ros_ws/src/Active-Road-Mapping/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
