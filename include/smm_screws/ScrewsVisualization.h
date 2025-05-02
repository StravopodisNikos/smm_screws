#ifndef SCREWS_VISUALIZATION_H
#define SCREWS_VISUALIZATION_H

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "ScrewsDynamics.h"

class ScrewsVisualization: public ScrewsDynamics {
public:
    ScrewsVisualization();
    ScrewsVisualization(RobotAbstractBase *ptr2abstract);
    ScrewsVisualization(RobotAbstractBase *ptr2abstract, ros::NodeHandle& nh);

    enum EllipsoidColor {
        ORANGE, // Kinematic
        OLIVE   // Dynamic
    };

    void publishTwists(Eigen::Matrix<float, 6, 1>* twists[robot_params::DOF]);
    void publishTwists(Eigen::Isometry3f* ptr2_active_tfs[robot_params::DOF+1]);
    void publishTCPpos(const Eigen::Vector3f& tcp_pos);
    void publishTCPvel(const Eigen::Vector3f& tcp_pos, const Eigen::Vector3f& tcp_vel);
    void publishKinematicManipulabilityEllipsoid(const Eigen::Vector3f& tcp_pos, const std::pair<Eigen::Matrix3f, Eigen::Vector3f>& ellipsoid_properties, EllipsoidColor color);
    void publishDynamicManipulabilityEllipsoid(const Eigen::Vector3f& tcp_pos, const std::pair<Eigen::Matrix3f, Eigen::Vector3f>& ellipsoid_properties, EllipsoidColor color);
private:
    ros::Publisher twist_pub;
    ros::Publisher tcp_pos_pub;
    ros::Publisher tcp_vel_pub;
    ros::Publisher kin_ell_pub;
    ros::Publisher kin_ell_axes_pub;
    ros::Publisher dyn_ell_pub;
    ros::Publisher dyn_ell_axes_pub;    
};

#endif // SCREWS_VISUALIZATION_H