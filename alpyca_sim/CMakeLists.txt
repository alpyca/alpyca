cmake_minimum_required(VERSION 2.8.3)
project(alpyca_sim)

find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  pybind11_catkin
)

catkin_python_setup()

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS pybind11_catkin
)

find_package(gazebo REQUIRED)

find_package(PythonLibs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -fvisibility=hidden")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
  LIBRARIES ${PROJECT_NAME}  
)

set(pybind_msgs "${CMAKE_CURRENT_BINARY_DIR}/proto/msgs.cpp")
add_custom_command(
    OUTPUT ${pybind_msgs}
    COMMAND python build/pybind/proto_to_pybind11.py /usr/include/gazebo-7/gazebo/msgs/proto ${pybind_msgs}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

pybind_add_module(py_contact_sensor MODULE src/alpyca/sim/py_contact_sensor.cpp)
pybind_add_module(msgs MODULE ${pybind_msgs})

add_library(plugin_runner SHARED src/alpyca/sim/plugin_runner.cpp)
target_link_libraries(plugin_runner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})
