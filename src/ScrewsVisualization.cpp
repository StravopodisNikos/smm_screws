#include "smm_screws/ScrewsVisualization.h"

ScrewsVisualization::ScrewsVisualization(RobotAbstractBase *ptr2abstract)
    : ScrewsDynamics(ptr2abstract)
{
    // Default constructor without ROS NodeHandle
}

ScrewsVisualization::ScrewsVisualization(RobotAbstractBase *ptr2abstract, ros::NodeHandle& nh)
    : ScrewsDynamics(ptr2abstract)
{
    twist_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_twists", 10);
    tcp_pos_pub = nh.advertise<visualization_msgs::Marker>("visualization_tcp_pos", 10);
}

void ScrewsVisualization::publishTwists(const Eigen::Matrix<float, 6, 1>* twists, size_t dof) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < dof; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "twists";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Define the start and end points of the arrow
        geometry_msgs::Point start, end;
        start.x = 0;
        start.y = 0;
        start.z = 0;
        //end.x = twists ;
        //end.y = twists ;
        //end.z = twists ;

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.1; // Shaft diameter
        marker.scale.y = 0.2; // Head diameter
        marker.scale.z = 0.2; // Head length

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    twist_pub.publish(marker_array);
}

void ScrewsVisualization::publishTCPpos(const Eigen::Vector3f& tcp_pos) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tcp_pos";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = tcp_pos.x();
    marker.pose.position.y = tcp_pos.y();
    marker.pose.position.z = tcp_pos.z();

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    tcp_pos_pub.publish(marker);
}
