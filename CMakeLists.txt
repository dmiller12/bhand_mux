cmake_minimum_required(VERSION 3.0.2)
project(bhand_mux)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      std_msgs
      visualization_msgs
      diagnostic_updater
      bhand_teleop_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
      std_msgs
      visualization_msgs
      bhand_teleop_msgs
      diagnostic_updater)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(bhand_mux
    src/bhand_mux.cpp
    src/bhand_mux_node.cpp
    src/bhand_mux_diagnostics.cpp)
target_link_libraries(bhand_mux ${catkin_LIBRARIES})

add_dependencies(bhand_mux ${catkin_EXPORTED_TARGETS})


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest(test/system.test)
endif()
