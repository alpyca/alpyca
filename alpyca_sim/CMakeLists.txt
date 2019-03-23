cmake_minimum_required(VERSION 2.8.3)
project(alpyca_sim)

find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

catkin_python_setup()

find_package(pybind11)
find_package(gazebo REQUIRED)
find_package(PythonLibs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -fvisibility=hidden")

set(bindings "${CMAKE_CURRENT_BINARY_DIR}/bindings")
set(bindings_includes "${bindings}/includes")
set(bindings_sensors "${bindings}/sensors")

file(MAKE_DIRECTORY ${bindings})
file(MAKE_DIRECTORY ${bindings_includes})
file(MAKE_DIRECTORY ${bindings_sensors})

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${bindings_includes} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
  LIBRARIES ${PROJECT_NAME} pybind11
)

set(pybind_msgs "${CMAKE_CURRENT_BINARY_DIR}/proto/msgs.cpp")
add_custom_command(
    OUTPUT ${pybind_msgs}
    COMMAND python build/pybind/proto_to_pybind11.py /usr/include/gazebo-7/gazebo/msgs/proto ${pybind_msgs}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Replace semicolon with comma, because a semicolon is interpreted as end of line
string(REPLACE ";" "," GAZEBO_INCLUDE_DIRS_COMMA_SEPARATED "${GAZEBO_INCLUDE_DIRS}")

set(contact_sensor_binding "${bindings_sensors}/py_contact_sensor.cpp")
set(contact_sensor_wrapper "${bindings_includes}/contact_sensor_wrapper.h")
add_custom_command(
    OUTPUT ${contact_sensor_binding} ${contact_sensor_wrapper}
    COMMAND python src/alpyca/sim/generate_sensors.py ${CMAKE_CURRENT_SOURCE_DIR}/src/alpyca/sim ${contact_sensor_binding} ${contact_sensor_wrapper} ${GAZEBO_INCLUDE_DIRS_COMMA_SEPARATED}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

macro(pybind_add_module target_name other)
    pybind11_add_module(${ARGV})
    target_link_libraries(${target_name} PRIVATE ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_GLOBAL_PYTHON_DESTINATION})
    set(PYTHON_LIB_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
    add_custom_command(TARGET ${target_name}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory ${PYTHON_LIB_DIR}
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target_name}> ${PYTHON_LIB_DIR}/${target_name}.so
      WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}
  COMMENT "Copying library files to python directory")
endmacro(pybind_add_module)

#pybind_add_module(py_contact_sensor MODULE src/alpyca/sim/py_contact_sensor.cpp)
pybind_add_module(py_contact_sensor MODULE ${contact_sensor_binding})
pybind_add_module(msgs MODULE ${pybind_msgs})

add_library(plugin_runner SHARED src/alpyca/sim/plugin_runner.cpp)
target_link_libraries(plugin_runner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})
