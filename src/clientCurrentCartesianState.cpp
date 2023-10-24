#include "ros/ros.h"
#include "smm_screws/SetCurrentCartesianState.h" 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_cartesian_state_client");
    ros::NodeHandle nh;

    // Create a client to call the service
    ros::ServiceClient client = nh.serviceClient<smm_screws::SetCurrentCartesianState>("current_cartesian_state_srv");

    // Create a service request
    smm_screws::SetCurrentCartesianState srv;
    srv.request.give_cur_cart_state = true;

    ros::Rate rate(10); // 10Hz
    while (ros::ok())
    {
        // Call the service
        if (client.call(srv))
        {
                // The service call was successful, and you can now access the Jacobian elements
                float p_qs_x = srv.response.p_e_s_x;
                float p_qs_y = srv.response.p_e_s_y;
                float p_qs_z = srv.response.p_e_s_z;
                float v_qs_x = srv.response.v_e_s_x;
                float v_qs_y = srv.response.v_e_s_y;
                float v_qs_z = srv.response.v_e_s_z;
                // Just print 
                ROS_INFO("Cartesian Position State [x] : %f ", p_qs_x);
                ROS_INFO("Cartesian Position State [y] : %f ", p_qs_y);
                ROS_INFO("Cartesian Position State [z] : %f ", p_qs_z);
                ROS_INFO("Cartesian Velocity State [x] : %f ", v_qs_x);
                ROS_INFO("Cartesian Velocity State [y] : %f ", v_qs_y);
                ROS_INFO("Cartesian Velocity State [z] : %f ", v_qs_z);

        }
        else
        {
            ROS_ERROR("[CLIENT-CurrentCartesianState] Failed to call: current_cartesian_state_srv.");
            return 1;
        }

        rate.sleep();
    }

    return 0;
}