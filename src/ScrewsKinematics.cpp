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
    _debug_verbosity = true;
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
    _gst = twistExp(_ptr2abstract->active_twists[0], *(q+0) ) * _Pi[0] * twistExp(_ptr2abstract->active_twists[1], *(q+1) ) * _Pi[1] * twistExp(_ptr2abstract->active_twists[2], *(q+2) ) * *gst_0 ;
    ROS_INFO("gst_x: %f", _gst(0,3));
    ROS_INFO("gst_y: %f", _gst(1,3));
    ROS_INFO("gst_z: %f", _gst(2,3));
    return;
}

void ScrewsKinematics::ForwardKinematics3DOF_1(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]) {
    // Returns a matrix of pointers of the robot's active joints' frames
    // @ the given configuration. input is the pointer to active joints'
    // frame @ zero config
    // "_1" -> Implements eq.8/p.44/[2]
    // Calculates the "Ci_1" matrices /in screw_dynamics/calculateFwdKinMueller.m [laptop]
    _debug_verbosity = false;

    // Calculate 1st joint frame
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[0]), _ptr2abstract->active_twists[0]);
    *gs_a_i[0] = *(_ptr2abstract->gsai_ptr[0]) *  twistExp(_X, q[0]);  
    _trans_vector = gs_a_i[0]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[1]), *(_ptr2abstract->gsai_ptr[0]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[1]), _ptr2abstract->active_twists[1]);
    *gs_a_i[1] = *gs_a_i[0] * _Bi * twistExp(_X, q[1]);
    _trans_vector = gs_a_i[1]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z()); 
    
    // Calculate 3rd joint frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[2]), *(_ptr2abstract->gsai_ptr[1]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[2]), _ptr2abstract->active_twists[2]);
    *gs_a_i[2] =  *gs_a_i[1] * _Bi * twistExp(_X, q[2]);
    _trans_vector = gs_a_i[2]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[3]), *(_ptr2abstract->gsai_ptr[2]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[2]), _ptr2abstract->active_twists[2]);
    *gs_a_i[3] =  *gs_a_i[2] * _Bi;  // this must be equal to _gst /in ForwardKinematicsTCP()
    _trans_vector = gs_a_i[3]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gst_x_1: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gst_y_1: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gst_z_1: %f", _trans_vector.z()); 
    
    return;
}

void ScrewsKinematics::ForwardKinematics3DOF_2(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]) {
    // Returns a matrix of pointers of the robot's active joints' frames
    // @ the given configuration. input is the pointer to active joints'
    // frame @ zero config
    // "_2" -> Implements eq.94/p.241/[3], "this is also the classic Murray Book equation"
    _debug_verbosity = false;

    // Calculate 1st joint frame
    *gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    _trans_vector = gs_a_i[0]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    *gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    _trans_vector = gs_a_i[1]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    *gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    _trans_vector = gs_a_i[2]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    *gs_a_i[3] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[3]) ;
    _trans_vector = gs_a_i[3]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gst_x_2: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gst_y_2: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gst_z_2: %f", _trans_vector.z()); 
    
    return;
}

