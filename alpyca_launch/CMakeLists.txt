cmake_minimum_required(VERSION 2.8.3)
project(alpyca_launch)

find_package(PythonLibs REQUIRED)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_install_python(PROGRAMS
    src/alpyca/__init__.py
    src/alpyca/launch/__init__.py
    src/alpyca/launch/launcher.py
    src/alpyca/launch/example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/aplyca)

