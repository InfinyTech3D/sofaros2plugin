
#include <sofa/core/ObjectFactory.h>

#include <string>

#define ROS2PLUGIN_DYNAMIC_LIBRARY __declspec(dllexport)

namespace sofa
{
namespace component
{

extern "C" {
    ROS2PLUGIN_DYNAMIC_LIBRARY void initExternalModule();
    ROS2PLUGIN_DYNAMIC_LIBRARY const char* getModuleName();
    ROS2PLUGIN_DYNAMIC_LIBRARY const char* getModuleVersion();
    ROS2PLUGIN_DYNAMIC_LIBRARY const char* getModuleLicense();
    ROS2PLUGIN_DYNAMIC_LIBRARY const char* getModuleDescription();
    ROS2PLUGIN_DYNAMIC_LIBRARY const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return "ROS2Plugin";
}

const char* getModuleVersion()
{
    return "0.0";
}

const char* getModuleLicense()
{
    return "Private";
}


const char* getModuleDescription()
{
    return "ROS2 and SOFA interfaces";
}

const char* getModuleComponentList()
{
    return "";
}

}  // namespace component
}  // namespace sofa
