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
    tcp_vel_pub = nh.advertise<visualization_msgs::Marker>("visualization_tcp_vel", 10);
    kin_ell_pub = nh.advertise<visualization_msgs::Marker>("visualization_kin_ell", 10);
    kin_ell_axes_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_kin_ell_axes", 10);
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
    
    Eigen::Matrix3f rotation_matrix;

    //Eigen::Vector4f active_tf_quaternion;  // Murray way
    Eigen::Quaternionf active_tf_quaternion; // Eigen built-in way

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
        //active_tf_quaternion = extractRotationQuaternion(*ptr2_active_tfs[i]);
        // Eigen built-in way
        if (i == 0) {
            //active_tf_quaternion = Eigen::Quaternionf::Identity(); // Quaternion for z-axis rotation
            active_tf_quaternion = Eigen::Quaternionf(Eigen::AngleAxisf(-M_PI / 2.0f, Eigen::Vector3f::UnitY())); // trick to visualize the joint 1 axis twist
        } else {
            // For other joints, extract and normalize rotation quaternion
            //rotation_matrix = ptr2_active_tfs[i]->rotation();
            //rotation_matrix.normalize();
            //active_tf_quaternion = Eigen::Quaternionf(rotation_matrix);
            active_tf_quaternion = Eigen::Quaternionf(ptr2_active_tfs[i]->rotation());
        }
        //rotation_matrix = ptr2_active_tfs[i]->rotation();
        //active_tf_quaternion = Eigen::Quaternionf(rotation_matrix);

        ROS_INFO("[publishTwists] Active Joint [%zu] Quaternion: W: %f, X: %f, Y: %f, Z: %f", i, active_tf_quaternion.w(), active_tf_quaternion.x(), active_tf_quaternion.y(), active_tf_quaternion.z());        

        // Murray way
        // Initialize orientation using the extracted quaternion
        //marker.pose.orientation.w = active_tf_quaternion(0);
        //marker.pose.orientation.x = active_tf_quaternion(1);
        //marker.pose.orientation.y = active_tf_quaternion(2);
        //marker.pose.orientation.z = active_tf_quaternion(3);
        // Eigen built-in way
        marker.pose.orientation.w = active_tf_quaternion.w();
        marker.pose.orientation.x = active_tf_quaternion.x();
        marker.pose.orientation.y = active_tf_quaternion.y();
        marker.pose.orientation.z = active_tf_quaternion.z();
        
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

void ScrewsVisualization::publishTCPvel(const Eigen::Vector3f& tcp_pos, const Eigen::Vector3f& tcp_vel) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tcp_vel";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Define the start point of the arrow (TCP position)
    marker.pose.position.x = tcp_pos.x();
    marker.pose.position.y = tcp_pos.y();
    marker.pose.position.z = tcp_pos.z();

    // Set the orientation of the marker
    Eigen::Quaternionf quaternion;
    quaternion.setFromTwoVectors(Eigen::Vector3f::UnitX(), tcp_vel.normalized());

    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    // Set the scale of the arrow
    marker.scale.x = tcp_vel.norm(); // Arrow length
    marker.scale.y = 0.20 * tcp_vel.norm(); // Arrow width
    marker.scale.z = 0.20 *  tcp_vel.norm(); // Arrow height

    // Set the color of the arrow (light purple)
    marker.color.r = 0.988;
    marker.color.g = 0.0;
    marker.color.b = 0.816;
    marker.color.a = 1.0;

    tcp_vel_pub.publish(marker);
}

void ScrewsVisualization::publishKinematicManipulabilityEllipsoid(const Eigen::Vector3f& tcp_pos, const std::pair<Eigen::Matrix3f, Eigen::Vector3f>& ellipsoid_properties) {
    // Extract the matrix and vector from the pair
    Eigen::Matrix3f orientation_matrix = ellipsoid_properties.first;
    Eigen::Vector3f axes_lengths = ellipsoid_properties.second;

    // Normalize the orientation matrix to ensure it's a valid rotation matrix
    orientation_matrix.col(0).normalize();
    orientation_matrix.col(1).normalize();
    orientation_matrix.col(2).normalize();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "kinematic_ellipsoid";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker based on the axes lengths
    marker.scale.x = axes_lengths(0) * 2.0f;
    marker.scale.y = axes_lengths(1) * 2.0f;
    marker.scale.z = axes_lengths(2) * 2.0f;

    // set for orange
    marker.color.r = 0.871;
    marker.color.g = 0.520;
    marker.color.b = 0.150;
    marker.color.a = 0.25;

    // Set the orientation of the ellipsoid using the rotation matrix
    Eigen::Quaternionf quaternion(orientation_matrix);
    quaternion.normalize();  // Ensure the quaternion is normalized
    marker.pose.orientation.w = quaternion.w();
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();

    // Position the ellipsoid at the TCP {T} frame origin
    marker.pose.position.x = tcp_pos.x();
    marker.pose.position.y = tcp_pos.y();
    marker.pose.position.z = tcp_pos.z();

    // Publish the ellipsoid marker
    kin_ell_pub.publish(marker);

    // Create and publish arrows for the axes
    visualization_msgs::MarkerArray arrow_markers;
    for (int i = 0; i < 3; ++i) {
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = "world";
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "kinematic_ellipsoid_axes";
        arrow_marker.id = i + 1;
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;

        // Define the start and end points of the arrow
        geometry_msgs::Point start, end;
        start.x = tcp_pos.x();
        start.y = tcp_pos.y();
        start.z = tcp_pos.z();
        end.x = tcp_pos.x() + axes_lengths(i) * orientation_matrix(0, i);
        end.y = tcp_pos.y() + axes_lengths(i) * orientation_matrix(1, i);
        end.z = tcp_pos.z() + axes_lengths(i) * orientation_matrix(2, i);

        arrow_marker.points.push_back(start);
        arrow_marker.points.push_back(end);

        arrow_marker.scale.x = 0.01f; // arrow length
        arrow_marker.scale.y = 0.005f; // arrow width
        arrow_marker.scale.z = 0.005f; // arrow height

        // Set different colors for each axis
        if (i == 0) {
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 0.0;
        } else if (i == 1) {
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 1.0;
            arrow_marker.color.b = 0.0;
        } else {
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 1.0;
        }
        arrow_marker.color.a = 1.0;

        arrow_markers.markers.push_back(arrow_marker);
    }

    // Publish the arrow markers
    kin_ell_axes_pub.publish(arrow_markers);
}