#pragma once
#include <ros2plugin/common/MessageWrapper.h>
#include <ros2plugin/common/types.h>
#include <sofa/core/visual/VisualParams.h>

#include <algorithm>
#include <cmath>

namespace sofa
{
namespace ros2
{
template <class DataTypes, class ROS2_MSG>
class MessageArrayWrapper
{
public:
    static inline void draw(const sofa::core::visual::VisualParams* /*vparams*/, const DataTypes& /*sofa_type*/, const double& /*scale*/,
                            const helper::vector<int>& /*indexes*/)
    {
    }
    static inline DataTypes toSofa(const ROS2_MSG& /*ros_msg*/, const helper::vector<int>& /*indexes*/)
    {
        msg_info("ROS2Plugin") << "in function toSofa: Types informed at template are unknown";
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/, const helper::vector<int>& /*indexes*/)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
    }
};

/**  Array Wrapppers  **************************************************************************************************
 *      SOFA         <===>          ROS2
 *      helper::vector<Rigid>      PoseArray
 */
template <>
inline void MessageArrayWrapper<helper::vector<Rigid>, PoseArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                           const helper::vector<Rigid>& vec3d, const double& scale,
                                                                           const helper::vector<int>& indexes)
{
    if (vec3d.size() == 0 || vec3d.size() < (*std::max_element(indexes.begin(), indexes.end()))) return;
    for (const auto& idx : indexes) MessageWrapper<Rigid, PoseMsg>::draw(vparams, vec3d[idx], scale);
}
template <>
inline helper::vector<Rigid> MessageArrayWrapper<helper::vector<Rigid>, PoseArrayMsg>::toSofa(const PoseArrayMsg& msg,
                                                                                              const helper::vector<int>& indexes)
{
    if (msg.poses.size() == 0 || msg.poses.size() < (*std::max_element(indexes.begin(), indexes.end()))) return helper::vector<Rigid>();
    helper::vector<Rigid> returnVec;
    for (const auto& idx : indexes) returnVec.push_back(MessageWrapper<Rigid, PoseMsg>::toSofa(msg.poses[idx]));
    return returnVec;
}
template <>
inline PoseArrayMsg MessageArrayWrapper<helper::vector<Rigid>, PoseArrayMsg>::toROS(const helper::vector<Rigid>& vec3d,
                                                                                    const helper::vector<int>& indexes)
{
    if (vec3d.size() == 0 || vec3d.size() < (*std::max_element(indexes.begin(), indexes.end()))) return PoseArrayMsg();
    auto points = PoseArrayMsg();
    for (const auto& idx : indexes)
    {
        points.poses.push_back(MessageWrapper<Rigid, PoseMsg>::toROS(vec3d[idx]));
    }

    return points;
}

/**  Array Wrapppers  **************************************************************************************************
 *      SOFA         <===>          ROS2
 *      helper::vector<Vec3d>       TrackerArrayMsg
 */
template <>
inline void MessageArrayWrapper<helper::vector<Vec3d>, TrackerArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                              const helper::vector<Vec3d>& vec3d, const double& scale,
                                                                              const helper::vector<int>& indexes)
{
    if (vec3d.size() == 0 || vec3d.size() < (*std::max_element(indexes.begin(), indexes.end()))) return;
    //    for (const auto& idx : indexes) MessageWrapper<Vec3d, PoseMsg>::draw(vparams, vec3d[idx], scale);
}
template <>
inline helper::vector<Vec3d> MessageArrayWrapper<helper::vector<Vec3d>, TrackerArrayMsg>::toSofa(const TrackerArrayMsg& msg,
                                                                                                 const helper::vector<int>& indexes)
{
    helper::vector<Vec3d> returnVec;
    std::map<int, helper::vector<Vec3d> > indexedVectors;

    for (unsigned i = 0; i < msg.ids.size(); i++)
    {
        indexedVectors[(int)log2(msg.ids[i])].push_back(Vec3d(msg.points[i].x, msg.points[i].y, msg.points[i].z));
    }

    unsigned acc = 0;
    for (auto i : indexedVectors)
    {
        if (std::count(indexes.begin(), indexes.end(), acc))
        {
            for (auto j : i.second) returnVec.push_back(j);
        }
        acc++;
    }
    return returnVec;
}

}  // namespace ros2
}  // namespace sofa