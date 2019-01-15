cmake_minimum_required(VERSION 2.8.3)
project(alpyca_comm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation 
  std_msgs)

catkin_python_setup()

add_service_files(DIRECTORY srv FILES ExampleSrv.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_install_python(PROGRAMS
    src/alpyca/__init__.py
    src/alpyca/comm/__init__.py
    src/alpyca/comm/communication.py
    src/alpyca/comm/example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/aplyca)

