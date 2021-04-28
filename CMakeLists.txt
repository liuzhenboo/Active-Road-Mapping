cmake_minimum_required(VERSION 2.8.3)
project(active_road_mapping)

# Policy CMP0043 introduced in cmake version 3.0 IGNORES the use of COMPILE_DEFINITIONS suffixed variables, e.g. COMPILE_DEFINITIONS_DEBUG
# Set to OLD behavior until minimum cmake version >= 2.8.10 (version that COMPILE_DEFINITIONS can be set by generator expressions instead)
if (POLICY CMP0043)
	cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0043)
# Policy CMP0042 introduced in cmake version 3.0 enables the use of @rpath in an install name via MACOSX_RPATH by default
# Set to OLD behavior so that all versions use the same behavior, or until minimum cmake version >= 2.8.12 (version where @rpath is available)
if (POLICY CMP0042)
	cmake_policy(SET CMP0042 OLD)
endif (POLICY CMP0042)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(PCL 1.7 REQUIRED)
#add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set
add_definitions(-std=c++03) # To include -march=native if set


find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


# message
find_package(catkin REQUIRED COMPONENTS 
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
             image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
             pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs
             genmsg stereo_msgs move_base_msgs image_geometry 
             message_generation
)
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
add_message_files(
  FILES
  Transform_matrix.msg)

  generate_messages(
   DEPENDENCIES
   std_msgs)   

  #  catkin_package(
  # CATKIN_DEPENDS 
  # message_runtime )

  catkin_package(
  INCLUDE_DIRS road_mapping/include
  LIBRARIES rtabmap_ros
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
                 pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs message_runtime 
                 stereo_msgs move_base_msgs image_geometry ${optional_dependencies}
  DEPENDS RTABMap OpenCV
)

########################################################################
## Build  mapping##
#########################################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/road_mapping/include/Utils
  ${CMAKE_CURRENT_SOURCE_DIR}/road_mapping/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
)
add_library(${PROJECT_NAME} SHARED
road_mapping/src/PC_Segment.cpp
road_mapping/src/Attitude.cpp
road_mapping/src/System.cpp
road_mapping/src/Utils/Utils_pcl.cpp
road_mapping/src/Utils/Utils_transform.cpp
road_mapping/src/Map.cpp
road_mapping/src/Cell.cpp
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${catkin_LIBRARIES}
)
 add_executable(road_mapping road_mapping/src/Node.cpp)
target_link_libraries(road_mapping 
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${catkin_LIBRARIES}
${PROJECT_NAME}
)
########################################################################
## Build  slam##
#########################################################################
include_directories(
${CMAKE_CURRENT_SOURCE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK/include
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK/include/CameraModels
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK/Examples/ROS/ORB_SLAM3
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK/Thirdparty/DBoW2/lib/libDBoW2.so
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK/Thirdparty/g2o/lib/libg2o.so
${CMAKE_CURRENT_SOURCE_DIR}/SDK/ORB_SLAM3-0.3-beta_SDK/lib/libORB_SLAM3.so
${catkin_LIBRARIES}
-lboost_system
)

add_executable(mono_Inertial
orbslam3_node/ros_mono_inertial.cc
)

target_link_libraries(mono_Inertial
${LIBS}
)

