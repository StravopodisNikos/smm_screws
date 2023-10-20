#include "ros/ros.h"
#include "smm_screws/SetOperationalSpaceJacobian.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_operational_space_jacobian_client");
    ros::NodeHandle nh;

    // Create a client to call the service
    ros::ServiceClient client = nh.serviceClient<smm_screws::SetOperationalSpaceJacobian>("current_operational_space_jacobian_srv");

    // Create a service request
    smm_screws::SetOperationalSpaceJacobian srv;

    // Call the service
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            // The service call was successful, and you can now access the Jacobian elements
            float Jop_00 = srv.request.Jop_00;
            float Jop_01 = srv.request.Jop_01;
            float Jop_02 = srv.request.Jop_02;
            float Jop_10 = srv.request.Jop_10;
            float Jop_11 = srv.request.Jop_11;
            float Jop_12 = srv.request.Jop_12;
            float Jop_20 = srv.request.Jop_20;
            float Jop_21 = srv.request.Jop_21;
            float Jop_22 = srv.request.Jop_22;
            // Just print 
            ROS_INFO("Jop_00: %f", Jop_00);
            ROS_INFO("Jop_01: %f", Jop_01);
            ROS_INFO("Jop_02: %f", Jop_02);
            ROS_INFO("Jop_10: %f", Jop_10);
            ROS_INFO("Jop_11: %f", Jop_11);
            ROS_INFO("Jop_12: %f", Jop_12);
            ROS_INFO("Jop_20: %f", Jop_20);
            ROS_INFO("Jop_21: %f", Jop_21);
            ROS_INFO("Jop_22: %f", Jop_22);
        }
        else
        {
            ROS_ERROR("[Operational Space Jacobian Client] Service call failed.");
        }
    }
    else
    {
        ROS_ERROR("Failed to call Operational Space Jacobian service");
        return 1;
    }

    return 0;
}