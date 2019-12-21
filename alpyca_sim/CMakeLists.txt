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

# Search for the gazebo include directory, e.g.: /usr/include/gazebo-7
foreach(INCLUDE_DIR ${GAZEBO_INCLUDE_DIRS})
  if(IS_DIRECTORY ${INCLUDE_DIR}/gazebo/sensors)
    set(gazebo_include_dir ${INCLUDE_DIR})
  endif()
endforeach()

if(NOT DEFINED gazebo_include_dir)
  message(FATAL_ERROR "Could not find Gazebo include directory!" )
endif()

add_custom_command(
    OUTPUT ${pybind_msgs}
    COMMAND python build/pybind/proto_to_pybind11.py ${gazebo_include_dir}/gazebo/msgs/proto ${pybind_msgs}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

set(bindings_sensors "${bindings}/sensors.cpp")
add_custom_command(
    OUTPUT ${bindings_sensors}
    COMMAND python src/alpyca/sim/generate_sensors.py
      ${CMAKE_CURRENT_SOURCE_DIR}/src/alpyca/sim
      ${bindings_sensors}
      ${bindings_includes}
      ${gazebo_include_dir}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

pybind_add_module(sensors MODULE ${bindings_sensors})
pybind_add_module(msgs MODULE ${pybind_msgs})
pybind_add_module(element MODULE src/alpyca/sim/element.cpp)

add_library(plugin_runner SHARED src/alpyca/sim/plugin_runner.cpp)
target_link_libraries(plugin_runner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})
