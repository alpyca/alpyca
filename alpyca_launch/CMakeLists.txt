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

add_service_files(FILES
	StartStop.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_install_python(PROGRAMS
    src/alpyca/__init__.py
    src/alpyca/launch/__init__.py
    src/alpyca/launch/launch.py
    src/alpyca/launch/master.py
    src/alpyca/launch/node.py
    src/alpyca/launch/runner.py
    src/alpyca/launch/example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/aplyca)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_arg.py)
  catkin_add_nosetests(test/test_basic.py)
  catkin_add_nosetests(test/test_env.py)
  catkin_add_nosetests(test/test_if_unless.py)
  catkin_add_nosetests(test/test_include.py)
  catkin_add_nosetests(test/test_node.py)
  catkin_add_nosetests(test/test_param.py)
  catkin_add_nosetests(test/test_remap.py)
  catkin_add_nosetests(test/test_rosparam.py)
  catkin_add_nosetests(test/test_subst.py)
endif()
