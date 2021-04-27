cmake_minimum_required(VERSION 2.8.3)
project(mav_find_road)

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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
             image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
             pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs
             genmsg stereo_msgs move_base_msgs image_geometry 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_ros
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
                 pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs
                 stereo_msgs move_base_msgs image_geometry ${optional_dependencies}
  DEPENDS RTABMap OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils
  ${CMAKE_CURRENT_SOURCE_DIR}/include
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
src/PC_Segment.cpp
src/Attitude.cpp
src/System.cpp
src/Utils/Utils_pcl.cpp
src/Utils/Utils_transform.cpp
src/Map.cpp
src/Cell.cpp
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${catkin_LIBRARIES}
)
 add_executable(mav_find_roads src/Node.cpp)
target_link_libraries(mav_find_roads 
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${catkin_LIBRARIES}
${PROJECT_NAME}
)

