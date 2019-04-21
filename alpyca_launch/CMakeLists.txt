cmake_minimum_required(VERSION 2.8.3)
project(alpyca_launch)

find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(FILES
	NodeState.msg
	State.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_install_python(PROGRAMS
    src/alpyca/__init__.py
    src/alpyca/launch/__init__.py
    src/alpyca/launch/launcher.py
    src/alpyca/launch/master.py
    src/alpyca/launch/node.py
    src/alpyca/launch/example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/aplyca)

