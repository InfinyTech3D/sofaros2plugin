cmake_minimum_required(VERSION 3.12)
project(ros2plugin)
set(CMAKE_CXX_STANDARD 17)
#add_compile_options(-Wall -Wextra -Wpedantic)

#set(SOFA_INSTALL_DIR "$ENV{SOFA_INSTALL_DIR}" CACHE PATH "Path to install directory")

#if ("${SOFA_INSTALL_DIR}" STREQUAL "")
#    message(FATAL_ERROR "Please specify variable SOFA_INSTALL_DIR in your environnement")
#endif()
#list(APPEND CMAKE_PREFIX_PATH ${SOFA_INSTALL_DIR})

# SOFA packages containing our dependencies
find_package(Sofa.Framework REQUIRED) # Needed to use SofaCore, SofaHelper and SofaDefaultType
find_package(Sofa.Component.Topology.Container)
find_package(Sofa.Component.StateContainer)
find_package(Sofa.Component)
find_package(Sofa.GL.Component)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


set(CONFIG_FILES
        src/ros2plugin/initROS2Plugin.cpp
)

get_property(SOFA_SOURCE_DIR_ALL TARGET Sofa.Framework PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
list(GET SOFA_SOURCE_DIR_ALL 0 SOFA_SOURCE_DIR)

# Our nice new SOFA Components
file(GLOB_RECURSE HEADER_FILES
        "src/*.h"
        "src/*.inl"
        )

file(GLOB_RECURSE SOURCE_FILES
        "src/*.cpp"
        )

# SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${CONFIG_FILES} ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_DIR=\"${SofaFramework_DIR}/..\" -DSOFA_SOURCE_DIR=\"${SOFA_SOURCE_DIR}\" ")

### Dependencies
### - ROS2 Standard libraries
include_directories(src)
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${trajectory_msgs_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
    Sofa.Framework
    Sofa.Component.Topology.Container
    Sofa.Component.StateContainer
    Sofa.GL.Component
    Sofa.Component
    Boost::thread
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${trajectory_msgs_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs trajectory_msgs sensor_msgs)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

