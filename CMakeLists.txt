cmake_minimum_required(VERSION 3.12)
project(ros2plugin)

list(APPEND CMAKE_PREFIX_PATH ${SOFA_INSTALL_DIRECTORY})
list(APPEND CMAKE_PREFIX_PATH ${SOFA_PLUGIN_DIRECTORY})

#set(CMAKE_INSTALL_PREFIX ${SOFA_INSTALL_DIRECTORY}/plugins CACHE PATH "Install path prefix, prepended onto install directories." FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${SOFA_INSTALL_DIRECTORY}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${SOFA_INSTALL_DIRECTORY}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SOFA_INSTALL_DIRECTORY}/bin)

# SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED) # Needed to use SofaCore, SofaHelper and SofaDefaultType
find_package(OpenCVPlugin REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

set(required_dependencies
        "rclcpp"
        "std_msgs"
        "optitrack_msgs"
)
ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
find_package(optitrack_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(image_transport REQUIRED)


# Config files (tell SOFA this is a plugin)
set(CONFIG_FILES
        src/ros2plugin/initROS2Plugin.cpp
)

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
ament_target_dependencies(${PROJECT_NAME} ${required_dependencies})

### Dependencies
### - ROS2 Standard libraries
#include_directories(${rclcpp_INCLUDE_DIRS})
#include_directories(${std_msgs_INCLUDE_DIRS})
#include_directories(${sensor_msgs_INCLUDE_DIRS})
#include_directories(${geometry_msgs_INCLUDE_DIRS})
#include_directories(${optitrack_msgs_INCLUDE_DIRS})
#include_directories(${cv_bridge_INCLUDE_DIRS})
#include_directories(${image_transport_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
    SofaFramework
    OpenCVPlugin
#    ${rclcpp_LIBRARIES}
#    ${std_msgs_LIBRARIES}
#    ${sensor_msgs_LIBRARIES}
#    ${geometry_msgs_LIBRARIES}
#    ${optitrack_msgs_LIBRARIES}
    ${cv_bridge_LIBRARIES}
#    ${image_transport_LIBRARIES}
)

### Change default install prefix
#set(CMAKE_INSTALL_PREFIX ${SOFA_INSTALL_DIR}/plugins/${PROJECT_NAME} CACHE PATH "Install path prefix, prepended onto install directories." FORCE)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${SOFA_INSTALL_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${SOFA_INSTALL_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SOFA_INSTALL_DIR}/bin)

sofa_create_package_with_targets(
        PACKAGE_NAME ${PROJECT_NAME}
        PACKAGE_VERSION 0.1
        TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
        INCLUDE_SOURCE_DIR "src"
        INCLUDE_INSTALL_DIR ${PROJECT_NAME}
        RELOCATABLE "plugins"
)

ament_export_dependencies(${required_dependencies})
ament_auto_package()
