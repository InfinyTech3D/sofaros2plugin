#pragma once

#include <sofa/config.h>

#ifdef SOFA_BUILD_ROS2Plugin
#  define ROS2Plugin_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define ROS2Plugin_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif