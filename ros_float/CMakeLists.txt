cmake_minimum_required(VERSION 2.8.3)
project(ros_float)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(rosbag REQUIRED)
include_directories(${rosbag_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  grid_map_msgs
  grid_map_ros
  pcl_ros
  pcl_conversions

)

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)
#include_directories(${BFL_INCLUDE_DIRS}/bfl)
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})


#  This structure is valid if there are no tree nodes (e.g. the msg folder only)
#  add_message_files(
#  This structure is valid if there are no tree nodes (e.g. the msg folder only)
#  FILES
#  pressure.msg
#  temperature.msg
#  depth.msg
#  )


# This sgructure is valid if there are tree nodes (such as Keller_mesg, State_msgs etc)
add_message_files(
  DIRECTORY
    msg/keller_message
  FILES
    pressure.msg
    temperature.msg
    depth.msg
    )

add_message_files(
  DIRECTORY
    msg/gps_message
  FILES
    cmg.msg
    gps_timestamp.msg
    latitude.msg
    longitude.msg
    mag_var.msg
    sog.msg
    status.msg
    )

add_message_files(
    DIRECTORY
    msg/ctl_state_message
    FILES
    altitudeState.msg
    ctl_force.msg
    ctl_force_high.msg
    ctl_force_low.msg
    ctl_mode.msg
    ctl_status.msg
    depthFromState.msg
    in_water.msg
    thrust_cmd.msg
    traj_vel.msg
    volume_cmd.msg
    volumeMode_cmd.msg
    volumeRate_cmd.msg
    zvelocity.msg
    )

add_message_files(
    DIRECTORY
    msg/ctl_command_message
    FILES
    abort.msg
    actuator_mode.msg
    alt_ref.msg
    altitude_check_depth.msg
    ctl_active.msg
    ctl_modeFromCommand.msg
    depth_ref.msg
    est_water_depth.msg
    fake_altimeter.msg
    neutral_volume.msg
    thrust_offset.msg
    thrust_ref.msg
    vel_ref.msg
    volume_ref.msg
    )

add_message_files(
    DIRECTORY
    msg/alt_data_message
    FILES
    altitude.msg
    good.msg
    )

add_message_files(
    DIRECTORY
    msg/usbl_message
    FILES
    accuracy.msg
    ctime.msg
    depthFromUSBL.msg
    e.msg
    h.msg
    integrity.msg
    mtime.msg
    n.msg
    p.msg
    prop_time.msg
    r.msg
    remote_id.msg
    rssi.msg
    u.msg
    x.msg
    y.msg
    z.msg
    )

add_message_files(
    DIRECTORY
    msg/usbl_relative_ref_message
    FILES
    accuracy_from_rel_usbl.msg
    depth_rel_reader.msg
    remote_id_rel_usbl_reader.msg
    ship_heading.msg
    ship_latitude.msg
    ship_longitude.msg
    ship_pitch.msg
    ship_roll.msg
    target_x.msg
    target_y.msg
    target_z.msg
    )

add_message_files(
    DIRECTORY
    msg/cam_message
    FILES
    alt.msg
    distance.msg
    dt.msg
    imnum.msg
    stdalt.msg
    stddistance.msg
    std_X_distance.msg
    std_Y_distance.msg
    surveytime.msg
    xdistance.msg
    ydistance.msg
    )

add_message_files(
    DIRECTORY
    msg/uncertainty_preprocessor_messages
    FILES
    preproc_depth_with_uncertainty.msg
    preproc_pressure_with_uncertainty.msg
    preproc_temperature_with_uncertainty.msg
    )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_float
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime tf2 tf2_ros grid_map_msgs grid_map_ros
#  DEPENDS system_lib
)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
#find all the qt UI stuff
file(GLOB UI
    "include/filter_interface/*.ui"
)

message("Adding filter_interface UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (MB_UI_HDRS  ${UI})

message("Generating filter_interface UI Headers: " )
foreach(source ${MB_UI_HDRS})
    message("  - " ${source})
endforeach(source)




###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
)

file(GLOB INCLUDES
    "include/${PROJECT_NAME}/*.h"
    "include/${PROJECT_NAME}/*.cpp"
    "include/${PROJECT_NAME}/*.hpp"

    "include/${PROJECT_NAME}/imu_filter_madgwick/*.h"
    "include/${PROJECT_NAME}/imu_filter_madgwick/*cpp"
    "include/${PROJECT_NAME}/imu_filter_madgwick/hpp"
    )


file(GLOB SRCS
    "include/filter_interface/*.h"
    "include/filter_interface/*.cpp"
    "include/filter_interface/*.hpp"
)

file(GLOB QT_SRCS
    "include/filter_interface/*.h"
    "include/filter_interface/*.cpp"
    "include/filter_interface/*.hpp"
)


add_executable(floatlog2bag src/floatlog2bag.cpp ${INCLUDES})
target_link_libraries(floatlog2bag
${catkin_LIBRARIES}
${rosbag_LIBRARIES}
)

add_executable(unc_preprocessor_node src/unc_preprocessor_node.cpp ${INCLUDES})
target_link_libraries(unc_preprocessor_node
${catkin_LIBRARIES}
${rosbag_LIBRARIES}
)

add_executable(utm_node src/utm_node.cpp ${INCLUDES})
target_link_libraries(utm_node
${catkin_LIBRARIES}
${rosbag_LIBRARIES}
)


add_executable(pf_node include/ros_pf/particle_filetr_param.cpp
                           src/pf_node.cpp ${INCLUDES})
target_link_libraries(pf_node
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
)

add_executable(pf_first_order_node include/ros_pf/pf_first_order.cpp
                           src/pf_first_order_node.cpp ${INCLUDES})
target_link_libraries(pf_first_order_node
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
)

add_executable(pf_second_order_node include/ros_pf/pf_second_order.cpp
                           src/pf_second_order_node.cpp ${INCLUDES})
target_link_libraries(pf_second_order_node
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
)

add_executable(pf_second_order_bis_node include/ros_pf/pf_second_order_bis.cpp
                           src/pf_second_order_bis_node.cpp ${INCLUDES})
target_link_libraries(pf_second_order_bis_node
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
)

add_executable(pf_second_order_water_depth_node src/pf_second_order_water_depth_node.cpp
    include/ros_pf/pf_second_order_water_depth.cpp ${INCLUDES})

target_link_libraries(pf_second_order_water_depth_node
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
)


add_executable(pf_third_order_node include/ros_pf/pf_third_order.cpp
                           src/pf_third_order_node.cpp ${INCLUDES})
target_link_libraries(pf_third_order_node
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
)

#add_executable(ekf_water_depth_node src/ekf_water_depth_node.cpp
#    include/ros_ekf/ekf_water_depth.cpp ${INCLUDES})

#target_link_libraries(ekf_water_depth_node
#  ${catkin_LIBRARIES}
#  ${BFL_LIBRARIES}
#)



add_executable(particlefiltermaingui include/filter_interface/particlefiltermaingui.cpp ${MB_UI_HDRS} ${QT_SRCS} ${SRCS} )
target_link_libraries(particlefiltermaingui  Qt5::Widgets  Qt5::PrintSupport Qt5::Core ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
