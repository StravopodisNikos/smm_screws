#include "smm_screws/ScrewsKinematics.h"

ScrewsKinematics::ScrewsKinematics() {};

//ScrewsKinematics::ScrewsKinematics(const std::shared_ptr<RobotAbstractBase>& robot_def)
//    : _robot_def(robot_def) {
ScrewsKinematics::ScrewsKinematics(RobotAbstractBase *ptr2abstract):  _ptr2abstract(ptr2abstract) {
    _total_pseudojoints = _ptr2abstract->get_STRUCTURE_ID();
    //ROS_INFO("pseudos_3: %d", _total_pseudojoints);
    _meta1_pseudojoints = _ptr2abstract->get_PSEUDOS_METALINK1();
    _meta2_pseudojoints = _ptr2abstract->get_PSEUDOS_METALINK2();
    //ROS_INFO("meta1_3: %d", _meta1_pseudojoints);
    //ROS_INFO("meta1_3: %d", _meta2_pseudojoints);
    _last_twist_cnt =0;
    _last_expo = Eigen::Isometry3f::Identity();
    _Pi[0] = Eigen::Isometry3f::Identity();
    _Pi[1] = Eigen::Isometry3f::Identity();
}

void ScrewsKinematics::extractPseudoTfs() {
// Executed during object creation, initializes the fixed transformations induces by anatomy metamorphosis
// [5-10-23] Current version, ONLY supports 3dof robot, with 2 metamorphic links.
    if (_total_pseudojoints == 0)
    {
        for (size_t i = 0; i < METALINKS; i++)
        {
            _Pi[i] = Eigen::Isometry3f::Identity();
        }
    } else if (_total_pseudojoints == 2) {
        for (size_t i = 0; i < METALINKS; i++)
        {
            if (i == 0) // 1st metamorphic link
            {
                _last_twist_cnt = 0;
                _last_expo = Eigen::Isometry3f::Identity();
                for (size_t j = 0; j < _meta1_pseudojoints; j++)
                {
                    _Pi[i] = _last_expo * twistExp(_ptr2abstract->get_PASSIVE_TWISTS(_last_twist_cnt), _ptr2abstract->get_PSEUDO_ANGLES(_last_twist_cnt));
                    _last_expo = _Pi[i];
                    _last_twist_cnt++;
                }
            } else if (i == 1) // 2nd metamorphic link
            {
                _last_expo = Eigen::Isometry3f::Identity();
                for (size_t j = 0; j < _meta2_pseudojoints; j++)
                {
                    _Pi[i] = _last_expo * twistExp(_ptr2abstract->get_PASSIVE_TWISTS(_last_twist_cnt),_ptr2abstract->get_PSEUDO_ANGLES(_last_twist_cnt));
                    _last_expo = _Pi[i];
                    _last_twist_cnt++;
                }            
            }  
        }
    }
}

void ScrewsKinematics::ForwardKinematicsTCP(float *q) {
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];
    //ROS_INFO("gst0_x: %f", (*gst_0)(0, 3));
    //ROS_INFO("gst0_y: %f", (*gst_0)(1, 3));
    //ROS_INFO("gst0_z: %f", (*gst_0)(2, 3));    
    _gst = twistExp(_ptr2abstract->active_twists[0], *(q+0) ) * _Pi[0] * twistExp(_ptr2abstract->active_twists[1], *(q+1) ) * _Pi[1] * twistExp(_ptr2abstract->active_twists[2], *(q+2) ) * *gst_0 ;
    ROS_INFO("gst_x: %f", _gst(0,3));
    ROS_INFO("gst_y: %f", _gst(1,3));
    ROS_INFO("gst_z: %f", _gst(2,3));
    return;
}