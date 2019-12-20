cmake_minimum_required(VERSION 2.8.3)
project(alpyca_sim)

find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  pybind11_catkin
)

catkin_python_setup()

find_package(gazebo REQUIRED)
find_package(PythonLibs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -fvisibility=hidden")

set(bindings "${CMAKE_CURRENT_BINARY_DIR}/bindings")
set(bindings_includes "${bindings}/includes")

file(MAKE_DIRECTORY ${bindings})
file(MAKE_DIRECTORY ${bindings_includes})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    gazebo_ros
    pybind11_catkin
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${bindings_includes} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

set(pybind_msgs "${CMAKE_CURRENT_BINARY_DIR}/proto/msgs.cpp")
add_custom_command(
    OUTPUT ${pybind_msgs}
    COMMAND python build/pybind/proto_to_pybind11.py /usr/include/gazebo-7/gazebo/msgs/proto ${pybind_msgs}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Replace semicolon with comma, because a semicolon is interpreted as end of line
string(REPLACE ";" "," GAZEBO_INCLUDE_DIRS_COMMA_SEPARATED "${GAZEBO_INCLUDE_DIRS}")

set(bindings_sensors "${bindings}/sensors.cpp")
add_custom_command(
    OUTPUT ${bindings_sensors}
    COMMAND python src/alpyca/sim/generate_sensors.py
      ${CMAKE_CURRENT_SOURCE_DIR}/src/alpyca/sim
      ${bindings_sensors}
      ${bindings_includes}
      ${GAZEBO_INCLUDE_DIRS_COMMA_SEPARATED}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

pybind_add_module(sensors MODULE ${bindings_sensors})
pybind_add_module(msgs MODULE ${pybind_msgs})
pybind_add_module(element MODULE src/alpyca/sim/element.cpp)

add_library(plugin_runner SHARED src/alpyca/sim/plugin_runner.cpp)
target_link_libraries(plugin_runner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})
