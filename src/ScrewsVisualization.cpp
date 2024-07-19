#include "smm_screws/ScrewsVisualization.h"

ScrewsVisualization::ScrewsVisualization()
    : ScrewsDynamics(nullptr)
{
    // Default constructor without ROS NodeHandle
}

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

void ScrewsVisualization::publishTwists(Eigen::Matrix<float, 6, 1>* twists[DOF]) {
    // Publishes active joint twists, extracted from the Spatial Jacobian columns
    visualization_msgs::MarkerArray marker_array;
    
    Eigen::Vector3f start_point_extracted;
    Eigen::Vector3f end_point_extracted;

    for (size_t i = 0; i < DOF; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "twists";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        // Extract the start-end point of each twist (Spatial Jacobian column vectors)
        extract_twist_points(*twists[i], start_point_extracted, end_point_extracted);
        ROS_INFO("Start Point [%zu]: X: %f, Y: %f, Z: %f", i, start_point_extracted.x(), start_point_extracted.y(), start_point_extracted.z());
        ROS_INFO("End Point [%zu]: X: %f, Y: %f, Z: %f", i, end_point_extracted.x(), end_point_extracted.y(), end_point_extracted.z());

        // Define the start and end points of the arrow
        geometry_msgs::Point start, end;
        start.x = start_point_extracted.x();
        start.y = start_point_extracted.y();
        start.z = start_point_extracted.z();
        end.x = end_point_extracted.x();
        end.y = end_point_extracted.y();
        end.z = end_point_extracted.z();

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.005; // line width, when TYPE = LINE_LIST
        //marker.scale.y = 0.05; // Head diameter
        //marker.scale.z = 0.025; // Head length

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    twist_pub.publish(marker_array);
}

void ScrewsVisualization::publishTwists(Eigen::Isometry3f* ptr2_active_tfs[DOF+1]) {
    // Publishes active joint twists, extracted from active tfs
    visualization_msgs::MarkerArray marker_array;
    
    Eigen::Vector4f active_tf_quaternion;
    //Eigen::Quaternionf active_tf_quaternion;
    Eigen::Vector3f active_tf_origin;

    for (size_t i = 0; i < DOF; ++i) { // last tf is tcp, not interested here!
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "twists";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        //ROS_INFO("[publishTwists] Active tf [%zu]", i); // Just to check if i have access to current g 
        //printIsometryMatrix(*ptr2_active_tfs[i]);        

        // ORIENTATION
        // Murray way
        active_tf_quaternion = extractRotationQuaternion(*ptr2_active_tfs[i]);
        // Eigen built-in way
        //Eigen::Matrix3f rotation_matrix = ptr2_active_tfs[i]->rotation();
        //active_tf_quaternion = Eigen::Quaternionf(rotation_matrix);

        ROS_INFO("[publishTwists] Active Joint [%zu] Quaternion: W: %f, X: %f, Y: %f, Z: %f", i, active_tf_quaternion.w(), active_tf_quaternion.x(), active_tf_quaternion.y(), active_tf_quaternion.z());        

        // Initialize orientation using the extracted quaternion
        if (i==0) {
            marker.pose.orientation.w = active_tf_quaternion(0);
            marker.pose.orientation.x = active_tf_quaternion(3);
            marker.pose.orientation.y = active_tf_quaternion(2);
            marker.pose.orientation.z = active_tf_quaternion(1);
        } else {
            marker.pose.orientation.w = active_tf_quaternion(0);
            marker.pose.orientation.x = active_tf_quaternion(1);
            marker.pose.orientation.y = active_tf_quaternion(2);
            marker.pose.orientation.z = active_tf_quaternion(3);
        }

        //marker.pose.orientation.w = active_tf_quaternion.w();
        //marker.pose.orientation.x = active_tf_quaternion.x();
        //marker.pose.orientation.y = active_tf_quaternion.y();
        //marker.pose.orientation.z = active_tf_quaternion.z();
        
        // POSITION
        active_tf_origin = ptr2_active_tfs[i]->translation(); 
        marker.pose.position.x = active_tf_origin.x();
        marker.pose.position.y = active_tf_origin.y();
        marker.pose.position.z = active_tf_origin.z();

        // ARROR PROPERTIES
        marker.scale.x = 0.25; // arrow length
        marker.scale.y = 0.0075; // arrow width
        marker.scale.z = 0.005; // arrow height

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

    // Initialize orientation to identity (no rotation)
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    tcp_pos_pub.publish(marker);
}
