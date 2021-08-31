#pragma once
#include <cv_bridge/cv_bridge.h>
#include <ros2plugin/common/types.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{
namespace ros2
{
template <class DataTypes, class ROS2_MSG>
class MessageWrapper
{
public:
    static inline void draw(const sofa::core::visual::VisualParams* /*vparams*/, const DataTypes& /*sofa_type*/, const double& /*scale*/) {}
    static inline DataTypes toSofa(const ROS2_MSG& /*ros_msg*/, double scale = 1.0 /*scale*/)
    {
        msg_info("ROS2Plugin") << "in function toSofa: Types informed at template are unknown";
        return DataTypes();
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/, double scale = 1.0 /*scale*/)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
        return ROS2_MSG();
    }
};

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *     double                    Float64Msg
 */
template <>
inline double MessageWrapper<double, Float64Msg>::toSofa(const Float64Msg& ros_msg, double scale)
{
    return ros_msg.data * scale;
}
template <>
inline Float64Msg MessageWrapper<double, Float64Msg>::toROS(const double& sofa_type, double scale)
{
    auto ros_msg = Float64Msg();
    ros_msg.data = sofa_type * scale;
    return ros_msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *     DoubleArray               Float64ArrayMsg
 */
template <>
inline DoubleArray MessageWrapper<DoubleArray, Float64ArrayMsg>::toSofa(const Float64ArrayMsg& ros_msg, double scale)
{
    DoubleArray temp(ros_msg.data.begin(), ros_msg.data.end());
    for (double& i : temp)
    {
        i *= scale;
    }

    return temp;
}
template <>
inline Float64ArrayMsg MessageWrapper<DoubleArray, Float64ArrayMsg>::toROS(const DoubleArray& sofa_type, double scale)
{
    auto ros_msg = Float64ArrayMsg();
    ros_msg.data = sofa_type;
    for (double& i : ros_msg.data)
    {
        i *= scale;
    }
    return ros_msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Vec3d                      PointMsg
 */
template <>
inline void MessageWrapper<Vec3d, PointMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Vec3d& vec3d, const double& scale)
{
    vparams->drawTool()->drawSphere(vec3d, scale, RGBAColor(1, 1, 1, 1));
}
template <>
inline Vec3d MessageWrapper<Vec3d, PointMsg>::toSofa(const PointMsg& point, double scale)
{
    return Vec3d(point.x, point.y, point.z) * scale;
}
template <>
inline PointMsg MessageWrapper<Vec3d, PointMsg>::toROS(const Vec3d& vec3d, double scale)
{
    auto point = PointMsg();
    point.x = vec3d[0] * scale;
    point.y = vec3d[1] * scale;
    point.z = vec3d[2] * scale;
    return point;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Vec6d                      WrenchMsg
 */
template <>
inline void MessageWrapper<Vec6d, WrenchMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Vec6d& pointc, const double& scale)
{
}

template <>
inline Vec6d MessageWrapper<Vec6d, WrenchMsg>::toSofa(const WrenchMsg& point, double scale)
{
    Vec6d output;
    output[0] = point.wrench.force.x * scale;
    output[1] = point.wrench.force.y * scale;
    output[2] = point.wrench.force.z * scale;
    output[3] = point.wrench.torque.x * scale;
    output[4] = point.wrench.torque.y * scale;
    output[5] = point.wrench.torque.z * scale;
    return output;
}

template <>
inline WrenchMsg MessageWrapper<Vec6d, WrenchMsg>::toROS(const Vec6d& input, double scale)
{
    auto point = WrenchMsg();
    point.wrench.force.x = input[0] * scale;
    point.wrench.force.y = input[1] * scale;
    point.wrench.force.z = input[2] * scale;
    point.wrench.torque.x = input[3] * scale;
    point.wrench.torque.y = input[4] * scale;
    point.wrench.torque.z = input[5] * scale;
    return point;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Quat                      QuatMsg
 */
template <>
inline Quat MessageWrapper<Quat, QuatMsg>::toSofa(const QuatMsg& quat, double scale)
{
    return Quat(quat.x, quat.y, quat.z, quat.w);
}
template <>
inline QuatMsg MessageWrapper<Quat, QuatMsg>::toROS(const Quat& orientation, double scale)
{
    auto quat = QuatMsg();
    quat.x = orientation[0];
    quat.y = orientation[1];
    quat.z = orientation[2];
    quat.w = orientation[3];
    return quat;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Rigid                      PoseMsg
 */
template <>
inline void MessageWrapper<Rigid, PoseMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Rigid& pose, const double& scale)
{
    vparams->drawTool()->drawFrame(pose.getCenter(), pose.getOrientation(), scale * Vec3d(1, 1, 1));
}
template <>
inline Rigid MessageWrapper<Rigid, PoseMsg>::toSofa(const PoseMsg& pose, double scale)
{
    return Rigid(MessageWrapper<Vec3d, PointMsg>::toSofa(pose.position, scale), MessageWrapper<Quat, QuatMsg>::toSofa(pose.orientation));
}
template <>
inline PoseMsg MessageWrapper<Rigid, PoseMsg>::toROS(const Rigid& rigid, double scale)
{
    auto pose = PoseMsg();
    pose.position = MessageWrapper<Vec3d, PointMsg>::toROS(rigid.getCenter(), scale);
    pose.orientation = MessageWrapper<Quat, QuatMsg>::toROS(rigid.getOrientation());
    return pose;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Rigid                      PoseStampedMsg
 */
template <>
inline void MessageWrapper<Rigid, PoseStampedMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Rigid& pose, const double& scale)
{
    MessageWrapper<Rigid, PoseMsg>::draw(vparams, pose, scale);
}
template <>
inline Rigid MessageWrapper<Rigid, PoseStampedMsg>::toSofa(const PoseStampedMsg& msg, double scale)
{
    return MessageWrapper<Rigid, PoseMsg>::toSofa(msg.pose, scale);
}
template <>
inline PoseStampedMsg MessageWrapper<Rigid, PoseStampedMsg>::toROS(const Rigid& rigid, double scale)
{
    auto msg = PoseStampedMsg();
    msg.pose = MessageWrapper<Rigid, PoseMsg>::toROS(rigid, scale);
    msg.header.stamp =
        rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    return msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      CameraInfo                  CameraInfoMsg
 */
template <>
inline void MessageWrapper<CameraInfo, CameraInfoMsg>::draw(const sofa::core::visual::VisualParams* vparams, const CameraInfo& pose,
                                                            const double& scale)
{
}
template <>
inline CameraInfo MessageWrapper<CameraInfo, CameraInfoMsg>::toSofa(const CameraInfoMsg& msg, double scale)
{
    type::Mat3x4d temp;
    for (unsigned i = 0; i < 12; i++) temp(i / 4, i % 4) = msg.p[i];
    return CameraInfo(temp);
}
template <>
inline CameraInfoMsg MessageWrapper<CameraInfo, CameraInfoMsg>::toROS(const CameraInfo& rigid, double scale)
{
    auto msg = CameraInfoMsg();
    for (unsigned i = 0; i < 12; i++) msg.p[i] = rigid.getMatrix()(i / 4, i % 4);
    return msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      CameraInfo                  CameraInfoMsg
 */
template <>
inline void MessageWrapper<SofaTwist, TwistMsg>::draw(const sofa::core::visual::VisualParams* vparams, const SofaTwist& pose, const double& scale)
{
}
template <>
inline SofaTwist MessageWrapper<SofaTwist, TwistMsg>::toSofa(const TwistMsg& msg, double scale)
{
    SofaTwist returnVec;
    returnVec[0] = msg.twist.linear.x * scale;
    returnVec[1] = msg.twist.linear.y * scale;
    returnVec[2] = msg.twist.linear.z * scale;
    returnVec[3] = msg.twist.angular.x * scale;
    returnVec[4] = msg.twist.angular.y * scale;
    returnVec[5] = msg.twist.angular.z * scale;

    return returnVec;
}
template <>
inline TwistMsg MessageWrapper<SofaTwist, TwistMsg>::toROS(const SofaTwist& rigid, double scale)
{
    auto msg = TwistMsg();

    msg.twist.linear.x = rigid[0] * scale;
    msg.twist.linear.y = rigid[1] * scale;
    msg.twist.linear.z = rigid[2] * scale;
    msg.twist.angular.x = rigid[3] * scale;
    msg.twist.angular.y = rigid[4] * scale;
    msg.twist.angular.z = rigid[5] * scale;

    return msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *    DoubleArray               JointStateMsg
 * Note: At the moment, only position data is transmitted
 * TODO: Integrate another DataType which accounts for velocity and forces (MechanicalStates ?)
 */
template <>
inline DoubleArray MessageWrapper<DoubleArray, JointStateMsg>::toSofa(const JointStateMsg& joint_msg, double scale)
{
    return DoubleArray(joint_msg.position.begin(), joint_msg.position.end());
}
template <>
inline JointStateMsg MessageWrapper<DoubleArray, JointStateMsg>::toROS(const DoubleArray& array, double scale)
{
    auto joint_msg = JointStateMsg();
    joint_msg.name.resize(array.size());
    joint_msg.position.resize(array.size());
    for (size_t i = 0; i < array.size(); i++)
    {
        joint_msg.name[i] = "joint_a" + std::to_string(i);
        joint_msg.position[i] = array[i] * scale;
    }
    joint_msg.header.stamp = rclcpp::Time();
    return joint_msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      sofa::type::vector<Vec3d>       PointArrayMsg
 */
template <>
inline void MessageWrapper<sofa::type::vector<Vec3d>, PoseArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                          const sofa::type::vector<Vec3d>& vec3d, const double& scale)
{
    for (unsigned i = 0; i < vec3d.size(); i++) vparams->drawTool()->drawSphere(vec3d[i], scale, RGBAColor(1, 1, 1, 1));
}
template <>
inline sofa::type::vector<Vec3d> MessageWrapper<sofa::type::vector<Vec3d>, PoseArrayMsg>::toSofa(const PoseArrayMsg& points, double scale)
{
    sofa::type::vector<Vec3d> returnVec;

    for (unsigned i = 0; i < points.poses.size(); i++)
        returnVec.push_back(Vec3d(points.poses[i].position.x, points.poses[i].position.y, points.poses[i].position.z) * scale);

    return returnVec;
}
template <>
inline PoseArrayMsg MessageWrapper<sofa::type::vector<Vec3d>, PoseArrayMsg>::toROS(const sofa::type::vector<Vec3d>& vec3d, double scale)
{
    auto points = PoseArrayMsg();

    for (unsigned i = 0; i < vec3d.size(); i++)
    {
        PoseArrayMsg::_poses_type::value_type P;
        P.position.set__x(vec3d[i][0] * scale).set__y(vec3d[i][1] * scale).set__z(vec3d[i][2] * scale);
        points.poses.push_back(P);
    }

    return points;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      sofa::type::vector<Rigid>      PoseArray
 */
template <>
inline void MessageWrapper<sofa::type::vector<Rigid>, PoseArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                          const sofa::type::vector<Rigid>& vec3d, const double& scale)
{
    for (unsigned i = 0; i < vec3d.size(); i++) MessageWrapper<Rigid, PoseMsg>::draw(vparams, vec3d[i], scale);
}
template <>
inline sofa::type::vector<Rigid> MessageWrapper<sofa::type::vector<Rigid>, PoseArrayMsg>::toSofa(const PoseArrayMsg& msg, double scale)
{
    sofa::type::vector<Rigid> returnVec;

    for (unsigned i = 0; i < msg.poses.size(); i++) returnVec.push_back(MessageWrapper<Rigid, PoseMsg>::toSofa(msg.poses[i], scale));

    return returnVec;
}
template <>
inline PoseArrayMsg MessageWrapper<sofa::type::vector<Rigid>, PoseArrayMsg>::toROS(const sofa::type::vector<Rigid>& vec3d, double scale)
{
    auto points = PoseArrayMsg();

    for (unsigned i = 0; i < vec3d.size(); i++)
    {
        points.poses.push_back(MessageWrapper<Rigid, PoseMsg>::toROS(vec3d[i], scale));
    }

    return points;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      sofa::type::vector<SofaSphere>      SphereArrayMsg
 */
template <>
inline void MessageWrapper<sofa::type::vector<SofaSphere>, SphereArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                                 const sofa::type::vector<SofaSphere>& vec3d, const double& scale)
{
}

template <>
inline sofa::type::vector<SofaSphere> MessageWrapper<sofa::type::vector<SofaSphere>, SphereArrayMsg>::toSofa(const SphereArrayMsg& msg, double scale)
{
    sofa::type::vector<SofaSphere> returnVec;

    for (unsigned i = 0; i < msg.centers.size(); i++)
    {
        returnVec.push_back(SofaSphere(msg.centers[i].x, msg.centers[i].y, msg.centers[i].z, msg.radius[i]) * scale);
    }
    return returnVec;
}

template <>
inline SphereArrayMsg MessageWrapper<sofa::type::vector<SofaSphere>, SphereArrayMsg>::toROS(const sofa::type::vector<SofaSphere>& spheres,
                                                                                            double scale)
{
    SphereArrayMsg points = SphereArrayMsg();
    geometry_msgs::msg::Point32 temp;

    for (unsigned i = 0; i < spheres.size(); i++)
    {
        temp.x = spheres[i][0] * scale;
        temp.y = spheres[i][1] * scale;
        temp.z = spheres[i][2] * scale;
        points.ids.push_back(i);
        points.centers.push_back(temp);
        points.radius.push_back(spheres[i][3] * scale);
    }

    return points;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      ImageData                  ImageMsg
 */
template <>
inline void MessageWrapper<SofaImage, ImageMsg>::draw(const sofa::core::visual::VisualParams* vparams, const SofaImage& pose, const double& scale)
{
}
template <>
inline SofaImage MessageWrapper<SofaImage, ImageMsg>::toSofa(const ImageMsg& msg, double scale)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        msg_error("ROS2Plugin") << "cv_bridge exception: " << e.what();
        return SofaImage();
    }
    return SofaImage(cv_ptr->image);
}
template <>
inline ImageMsg MessageWrapper<SofaImage, ImageMsg>::toROS(const SofaImage& sofa_type, double scale)
{
    cv::Mat image = sofa_type.getImage();
    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = image.type();
    cv_image.header.stamp = rclcpp::Time();
    return *cv_image.toCompressedImageMsg();
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      ImageData                  GenericImageMsg
 */
template <>
inline void MessageWrapper<SofaImage, GenericImageMsg>::draw(const sofa::core::visual::VisualParams* vparams, const SofaImage& pose, const double& scale)
{
}
template <>
inline SofaImage MessageWrapper<SofaImage, GenericImageMsg>::toSofa(const GenericImageMsg& msg, double scale)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        msg_error("ROS2Plugin") << "cv_bridge exception: " << e.what();
        return SofaImage();
    }
    return SofaImage(cv_ptr->image);
}
template <>
inline GenericImageMsg MessageWrapper<SofaImage, GenericImageMsg>::toROS(const SofaImage& sofa_type, double scale)
{
    cv::Mat image = sofa_type.getImage();
    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = image.type();
    cv_image.header.stamp = rclcpp::Time();
    return *cv_image.toImageMsg();
}

}  // namespace ros2
}  // namespace sofa