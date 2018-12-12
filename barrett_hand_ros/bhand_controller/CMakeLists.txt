cmake_minimum_required(VERSION 2.8.3)
project(bhand_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  sensor_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   State.msg
   TactileArray.msg
   Service.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Actions.srv
   SetControlMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()

catkin_install_python(PROGRAMS src/bhand_controller/bhand_node.py src/bhand_controller/puck_properties_consts.py src/bhand_controller/puck_properties_consts.py src/bhand_controller/pyHand_api.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
