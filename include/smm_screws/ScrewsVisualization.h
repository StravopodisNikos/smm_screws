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

    void publishTwists(const Eigen::Matrix<float, 6, 1>* twists, size_t dof);
    void publishTCPpos(const Eigen::Vector3f& tcp_pos);
    
private:
    ros::Publisher twist_pub;
    ros::Publisher tcp_pos_pub;
};

#endif // SCREWS_VISUALIZATION_H