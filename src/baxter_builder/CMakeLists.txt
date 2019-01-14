cmake_minimum_required(VERSION 2.8.3)
project(baxter_builder)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    baxter_core_msgs
    baxter_interface
    baxter_maintenance_msgs
    control_msgs
    cv_bridge
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    trajectory_msgs
    moveit_core
    moveit_ros_planning_interface
    message_generation
    gazebo_ros
    gazebo_msgs
    baxter_tools
    baxter_gazebo
    roslaunch
)

add_service_files(
    FILES
    ObjLocation.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(
    CATKIN_DEPENDS
    actionlib
    baxter_core_msgs
    baxter_interface
    baxter_maintenance_msgs
    control_msgs
    cv_bridge
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    trajectory_msgs
    moveit_core
    moveit_ros_planning_interface
    message_generation
    gazebo_ros
    gazebo_msgs
    baxter_tools
    baxter_gazebo
    roslaunch
)

install(PROGRAMS
   scripts/baxter_mover_right_arm.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Check all launch files in the package
# When build catkin workspace, run:
# $ catkin_make run_tests
# to run all unit tests, such as the one below
# To get a summary of the unit test results:
# $catkin_test_results build/test_results
if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()






