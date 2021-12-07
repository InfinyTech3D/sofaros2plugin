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
                            const sofa::type::vector<int>& /*indexes*/)
    {
    }
    static inline DataTypes toSofa(const ROS2_MSG& /*ros_msg*/, const sofa::type::vector<int>& /*indexes*/,double /*scale*/=1.0)
    {
        msg_info("ROS2Plugin") << "in function toSofa: Types informed at template are unknown";
        return DataTypes();
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/, const sofa::type::vector<int>& /*indexes*/,double scale=1.0)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
    }
    template <class ARRAY>
    static bool isIndexValid(const ARRAY& array, const sofa::type::vector<int>& indexes)
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
                                                               const double& scale, const sofa::type::vector<int>& indexes)
{
    if (!isIndexValid(vec3d, indexes)) return;
    for (const auto& idx : indexes) MessageWrapper<Rigid, PoseMsg>::draw(vparams, vec3d[idx], scale);
}
template <>
inline PoseArray MessageArrayWrapper<PoseArray, PoseArrayMsg>::toSofa(const PoseArrayMsg& msg, const sofa::type::vector<int>& indexes,double scale)
{
    if (!isIndexValid(msg.poses, indexes)) return PoseArray();
    PoseArray returnVec;
    for (const auto& idx : indexes) returnVec.push_back(MessageWrapper<Rigid, PoseMsg>::toSofa(msg.poses[idx],scale));
    return returnVec;
}
template <>
inline PoseArrayMsg MessageArrayWrapper<PoseArray, PoseArrayMsg>::toROS(const PoseArray& vec3d, const sofa::type::vector<int>& indexes,double scale)
{
    if (!isIndexValid(vec3d, indexes)) return PoseArrayMsg();
    auto points = PoseArrayMsg();
    for (const auto& idx : indexes)
    {
        points.poses.push_back(MessageWrapper<Rigid, PoseMsg>::toROS(vec3d[idx],scale));
    }

    return points;
}

/**  Array Wrapppers  **************************************************************************************************
 *      SOFA         <===>          ROS2
 *      PointArray              TrackerArrayMsg
 */
template <>
inline PointArray MessageArrayWrapper<PointArray, TrackerArrayMsg>::toSofa(const TrackerArrayMsg& msg, const sofa::type::vector<int>& indexes,double scale)
{
    PointArray returnVec;
    std::map<int, PointArray> indexedVectors;

    for (unsigned i = 0; i < msg.ids.size(); i++)
    {
        indexedVectors[(msg.ids[i]>>16)].push_back(Vec3d(msg.points[i].x, msg.points[i].y, msg.points[i].z)*scale);
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

/**  Array Wrapppers  **************************************************************************************************
 *      SOFA         <===>          ROS2
 *      PointArray              TrackerArrayMsg
 */
template <>
inline PoseArray MessageArrayWrapper<PoseArray, RigidArrayMsg>::toSofa(const RigidArrayMsg& msg, const sofa::type::vector<int>& indexes,double scale)
{
    PoseArray returnVec;
    std::map<int, PointArray> indexedVectors;

    for (unsigned i = 0; i<msg.ids.size();i++)
    {

        if (std::count(indexes.begin(), indexes.end(),msg.ids[i]))
        {
            returnVec.push_back(MessageWrapper<Rigid, PoseMsg>::toSofa(msg.poses[i],scale));
        }
    }
    return returnVec;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *    DoubleArray               JointStateMsg
 * Note: At the moment, only position data is transmitted
 * TODO: Integrate another DataType which accounts for velocity and forces (MechanicalStates ?)
 */
template <>
inline DoubleArrayArray MessageArrayWrapper<DoubleArrayArray, JointStateMsg>::toSofa(const JointStateMsg& joint_msg, const sofa::type::vector<int>& indexes,double /*scale*/)
{
    if(!indexes.size())
        return DoubleArrayArray();

    DoubleArrayArray temp;

    for(auto i : indexes)
    {
        switch(i)
        {
            case 0:
                temp.push_back(DoubleArray(joint_msg.position.begin(), joint_msg.position.end()));
                break;
            case 1:
                temp.push_back(DoubleArray(joint_msg.velocity.begin(), joint_msg.velocity.end()));
                break;
            case 2:
                temp.push_back(DoubleArray(joint_msg.effort.begin(), joint_msg.effort.end()));
                break;
            default:
                break;
        }

    }

    return temp;
}
template <>
inline JointStateMsg MessageArrayWrapper<DoubleArrayArray, JointStateMsg>::toROS(const DoubleArrayArray& /*array*/, const sofa::type::vector<int>& /*indexes*/,double /*scale*/)
{
    std::cout<<"Method toROS for JointStateMsg to be coded if ever needed"<<std::endl;
    JointStateMsg msg;
    return msg;
}

}  // namespace ros2
}  // namespace sofa
