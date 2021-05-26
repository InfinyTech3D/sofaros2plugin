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
        return DataTypes();
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/, const helper::vector<int>& /*indexes*/)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
    }
    template <class ARRAY>
    static bool isIndexValid(const ARRAY& array, const helper::vector<int>& indexes)
    {
        unsigned max_index = (*std::max_element(indexes.begin(), indexes.end()));
        return (!array.empty() || max_index < array.size());  // true if not empty and index is below array size
    }
};

/**  Array Wrapppers  **************************************************************************************************
 *      SOFA         <===>          ROS2
 *      PoseArray                 PoseArray
 */
template <>
inline void MessageArrayWrapper<PoseArray, PoseArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams, const PoseArray& vec3d,
                                                               const double& scale, const helper::vector<int>& indexes)
{
    if (!isIndexValid(vec3d, indexes)) return;
    for (const auto& idx : indexes) MessageWrapper<Rigid, PoseMsg>::draw(vparams, vec3d[idx], scale);
}
template <>
inline PoseArray MessageArrayWrapper<PoseArray, PoseArrayMsg>::toSofa(const PoseArrayMsg& msg, const helper::vector<int>& indexes)
{
    if (!isIndexValid(msg.poses, indexes)) return PoseArray();
    PoseArray returnVec;
    for (const auto& idx : indexes) returnVec.push_back(MessageWrapper<Rigid, PoseMsg>::toSofa(msg.poses[idx]));
    return returnVec;
}
template <>
inline PoseArrayMsg MessageArrayWrapper<PoseArray, PoseArrayMsg>::toROS(const PoseArray& vec3d, const helper::vector<int>& indexes)
{
    if (!isIndexValid(vec3d, indexes)) return PoseArrayMsg();
    auto points = PoseArrayMsg();
    for (const auto& idx : indexes)
    {
        points.poses.push_back(MessageWrapper<Rigid, PoseMsg>::toROS(vec3d[idx]));
    }

    return points;
}

/**  Array Wrapppers  **************************************************************************************************
 *      SOFA         <===>          ROS2
 *      PointArray              TrackerArrayMsg
 */
template <>
inline PointArray MessageArrayWrapper<PointArray, TrackerArrayMsg>::toSofa(const TrackerArrayMsg& msg, const helper::vector<int>& indexes)
{
    PointArray returnVec;
    std::map<int, PointArray> indexedVectors;

    for (unsigned i = 0; i < msg.ids.size(); i++)
    {
        indexedVectors[(msg.ids[i]>>16)].push_back(Vec3d(msg.points[i].x, msg.points[i].y, msg.points[i].z));
    }

    for (auto i : indexedVectors)
    {

        if (std::count(indexes.begin(), indexes.end(), i.first))
        {
            for (auto j : i.second) returnVec.push_back(j);
        }
    }
    return returnVec;
}

}  // namespace ros2
}  // namespace sofa