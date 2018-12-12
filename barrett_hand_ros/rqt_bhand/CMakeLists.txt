cmake_minimum_required(VERSION 2.8.3)
project(rqt_bhand)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/rqt_bhand/rqt_bhand.py  src/rqt_bhand/tact_maps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
