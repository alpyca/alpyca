cmake_minimum_required(VERSION 2.8.3)
project(alpyca)

find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation 
  std_msgs
  gazebo_ros
  roscpp
  pybind11_catkin)

catkin_python_setup()

add_service_files(DIRECTORY srv FILES ExampleSrv.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS pybind11_catkin
)

find_package(gazebo REQUIRED)
find_package(PythonLibs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
  LIBRARIES ${PROJECT_NAME}  
)

pybind_add_module(py_sensor MODULE src/alpyca/sim/py_sensor.cpp)
pybind_add_module(py_contact_sensor MODULE src/alpyca/sim/py_contact_sensor.cpp)

add_library(plugin_runner SHARED src/alpyca/sim/plugin_runner.cpp)
target_link_libraries(plugin_runner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})


catkin_install_python(PROGRAMS
    src/alpyca/__init__.py
    src/alpyca/comm/__init__.py
    src/alpyca/comm/communication.py
    src/alpyca/comm/example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/aplyca)

