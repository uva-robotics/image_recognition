cmake_minimum_required(VERSION 2.8.3)
project(face_recognition_cv)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  src/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
