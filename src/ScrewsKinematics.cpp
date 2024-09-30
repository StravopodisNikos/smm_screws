#include "smm_screws/ScrewsKinematics.h"

ScrewsKinematics::ScrewsKinematics() {};

ScrewsKinematics::ScrewsKinematics(RobotAbstractBase *ptr2abstract):  _ptr2abstract(ptr2abstract) {
    _total_pseudojoints = _ptr2abstract->get_STRUCTURE_ID();
    _meta1_pseudojoints = _ptr2abstract->get_PSEUDOS_METALINK1();
    _meta2_pseudojoints = _ptr2abstract->get_PSEUDOS_METALINK2();
    _last_twist_cnt =0;
    _last_expo = Eigen::Isometry3f::Identity();
    _debug_verbosity = true;

    // Allocate memory for Jacobians [6x3]
        for (size_t i = 0; i < DOF; i++)
    {
        ptr2Jsp1[i] = &Jsp_t_1[i];
        ptr2Jsp2[i] = &Jsp_t_2[i];
        ptr2Jbd_t_1[i] = &Jbd_t_1[i];
        ptr2Jbd_t_2[i] = &Jbd_t_2[i];
        ptr2dJsp_t_1[i] = &dJsp_t_1[i];
        ptr2dJbd_t_1[i] = &dJbd_t_1[i];
        ptr2dJbd_t_2[i] = &dJbd_t_2[i];
    }
    for (int i = 0; i < DOF+1; ++i) {
        ptr2BodyJacobiansFrames[i] = new Eigen::Matrix<float, 6, 1>*[DOF];
        for (int j = 0; j < DOF; ++j) {
            ptr2BodyJacobiansFrames[i][j] = new Eigen::Matrix<float, 6, 1>;
        }
    }
    // Preallocate memory for tf  matrices used in internal calculations
    // for Mueller/Murray equations
    for (size_t i = 0; i < DOF+1; i++)
    {
        g[i]  = Eigen::Isometry3f::Identity();
        Bi[i] = Eigen::Isometry3f::Identity();
    }    
}

void ScrewsKinematics::initializeRelativeTfs() {
    // Initializes the matrices Bi ~ Ci_i1(0)
    _debug_verbosity = false;

    Bi[0] = *_ptr2abstract->gsai_ptr[0];
    if (_debug_verbosity) { printIsometryMatrix(Bi[0]); ROS_INFO(" ");}
    for (size_t i = 1; i < DOF+1; i++)
    {
        Bi[i] = extractRelativeTf(*_ptr2abstract->gsai_ptr[i], *_ptr2abstract->gsai_ptr[i-1]);
        if (_debug_verbosity) { printIsometryMatrix(Bi[i]); ROS_INFO(" ");}
    }
    return;
}

void ScrewsKinematics::initializeRelativeTfs(Eigen::Isometry3f* B_i[DOF+1]) {
    // Initializes the matrices Bi ~ Ci_i1(0)
    _debug_verbosity = false;

    B_i[0] = _ptr2abstract->gsai_ptr[0];
    if (_debug_verbosity) { printIsometryMatrix(*B_i[0]); ROS_INFO(" ");}
    for (size_t i = 1; i < DOF+1; i++)
    {
        *B_i[i] = extractRelativeTf(*_ptr2abstract->gsai_ptr[i], *_ptr2abstract->gsai_ptr[i-1]);
        if (_debug_verbosity) { printIsometryMatrix(*B_i[i]); ROS_INFO(" ");}
    }
    return;
}

void ScrewsKinematics::initializeLocalScrewCoordVectors(Eigen::Matrix<float, 6, 1> *i_X_i[DOF+1]) {
    // Initializes the matrices i_X_i;
    _debug_verbosity = false;

    for (size_t i = 0; i < DOF; i++){
        *i_X_i[i] = extractLocalScrewCoordVector(*_ptr2abstract->gsai_ptr[i], _ptr2abstract->active_twists[i]);
    }
    // For the tool frame, the relative tf must be extracted
    _Bi = extractRelativeTf(*_ptr2abstract->gsai_ptr[DOF], *_ptr2abstract->gsai_ptr[DOF-1]);
    vee(*i_X_i[DOF]  , _Bi.matrix() );
    //if (_debug_verbosity) {  print6nMatrix(i_X_i, DOF+1); ROS_INFO(" ");} 
    return;
}

void ScrewsKinematics::initializeLocalScrewCoordVectors() {
    for (size_t i = 0; i < DOF; i++){
        iXi[i] = extractLocalScrewCoordVector(*_ptr2abstract->gsai_ptr[i], _ptr2abstract->active_twists[i]);
    }
    // For the tool frame, the relative tf must be extracted
    _Bi = extractRelativeTf(*_ptr2abstract->gsai_ptr[DOF], *_ptr2abstract->gsai_ptr[DOF-1]);
    vee(iXi[DOF]  , _Bi.matrix() );
    return;
}

void ScrewsKinematics::initializePseudoTfs() {
// Executed during object creation, initializes the fixed transformations induces by anatomy metamorphosis
// [5-10-23] Current version, ONLY supports 3dof robot, with 2 metamorphic links.
// [13-7-24] Revised during major library shakeup. No need for else if.
    if (_total_pseudojoints == 0)
    {
        for (size_t i = 0; i < METALINKS; i++)
        {
            _Pi[i] = Eigen::Isometry3f::Identity();
        }
    } else {
        for (size_t i = 0; i < METALINKS; i++)
        {
            if (i == 0) // 1st metamorphic link
            {
                _last_twist_cnt = 0;
                _last_expo = Eigen::Isometry3f::Identity();
                for (size_t j = 0; j < _meta1_pseudojoints; j++)
                {
                    //printTwist(_ptr2abstract->get_PASSIVE_TWISTS(_last_twist_cnt));
                    _Pi[i] = _last_expo * twistExp(_ptr2abstract->get_PASSIVE_TWISTS(_last_twist_cnt), _ptr2abstract->get_PSEUDO_ANGLES(_last_twist_cnt));
                    _last_expo = _Pi[i];
                    _last_twist_cnt++;
                }
            } else if (i == 1) // 2nd metamorphic link
            {
                _last_expo = Eigen::Isometry3f::Identity();
                for (size_t j = 0; j < _meta2_pseudojoints; j++)
                {
                    //printTwist(_ptr2abstract->get_PASSIVE_TWISTS(_last_twist_cnt));
                    _Pi[i] = _last_expo * twistExp(_ptr2abstract->get_PASSIVE_TWISTS(_last_twist_cnt),_ptr2abstract->get_PSEUDO_ANGLES(_last_twist_cnt));
                    _last_expo = _Pi[i];
                    _last_twist_cnt++;
                }            
            }  
        }
    }
    return;
}

void ScrewsKinematics::initializeAnatomyActiveTwists() {
    // [13-7-24] Transformes the active twists provided in reference anatomy, to the
    //           anatomy specified by the pseudoangles in passive_definition.h ln.19
    //           These active twists should be used in dynamics and when used in kinematics
    //           NO pseudo expos must be used in the calculations!
    //           xa_ai_anat from MATLAB is extracted
    _debug_verbosity = false;

    _ptr2abstract->active_twists_anat[0] = _ptr2abstract->active_twists[0];
    
    ad(_ad, _Pi[0]);
    _ptr2abstract->active_twists_anat[1] = _ad * _ptr2abstract->active_twists[1];

    ad(_ad, _Pi[0] * _Pi[1]);
    _ptr2abstract->active_twists_anat[2] = _ad * _ptr2abstract->active_twists[2];    

    _ptr2abstract->ptr2_active_twists_anat[0] = &_ptr2abstract->active_twists_anat[0];
    _ptr2abstract->ptr2_active_twists_anat[1] = &_ptr2abstract->active_twists_anat[1];
    _ptr2abstract->ptr2_active_twists_anat[2] = &_ptr2abstract->active_twists_anat[2];

    if (_debug_verbosity)
    {
        print6nMatrix( _ptr2abstract->ptr2_active_twists_anat , DOF);
    }
    
    return;
}

void ScrewsKinematics::updateJointState(float *q_new, float *dq_new, float *ddq_new) {
    // Updates current position,velocity and acceleration. Called every time the 
    // subscriber to /"robot_ns"/joint_states + /"robot_ns"/joint_accelerations is called.
    for (size_t i = 0; i < DOF; i++) {
        _joint_pos[i] = q_new[i];      // Update current pos
        _joint_vel[i] = dq_new[i];     // Update current vel
        _joint_accel[i] = ddq_new[i];     // Update current accel
    } 
    return;
}

void ScrewsKinematics::updateJointState(float *q_new, float *dq_new) {
    // Updates current position,velocity and acceleration. Called every time 
    // subscriber to /"robot_ns"/joint_states is ONLY called.
    for (size_t i = 0; i < DOF; i++) {
        _joint_pos[i] = q_new[i];      // Update current pos
        _joint_vel[i] = dq_new[i];     // Update current vel
    } 
    return;
}

void ScrewsKinematics::extractPassiveTfs(Eigen::Isometry3f* passive_expos[METALINKS]) {
    // Returns pointer array of pseudo tfs. initializePseudoTfs() MUST be previously
    // executed.
    for (size_t i = 0; i < METALINKS; i++)
    {
        *passive_expos[i] = _Pi[i];
    }
    return;
}


void ScrewsKinematics::extractActiveTfs(float *q, Eigen::Isometry3f* active_expos[DOF]) {
    // Calculates the active joints exponentials. But accepts an array pointer. 
    // Data computed here can be accessed from functions of different class, 
    // using the pointer array!
    for (size_t i = 0; i < DOF; i++)
    {
        *active_expos[i] = twistExp(_ptr2abstract->active_twists[i], *(q+i) );
    }
    return;
}

void ScrewsKinematics::extractActiveTfsAnat(float *q, Eigen::Isometry3f* active_expos[DOF]) {
    // [14-7-24]  Calculates the active joints exponentials for the test anatomy. But  
    //            accepts an array pointer. Data computed here can be accessed from  
    //            functions of different class, using the pointer array!
    for (size_t i = 0; i < DOF; i++)
    {
        *active_expos[i] = twistExp(_ptr2abstract->active_twists_anat[i], *(q+i) );
    }
    return;
}

void ScrewsKinematics::ForwardKinematicsTCP(float *q) {
    _debug_verbosity = false;
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];   
    //_gst = twistExp(_ptr2abstract->active_twists[0], *(q+0) ) * _Pi[0] * twistExp(_ptr2abstract->active_twists[1], *(q+1) ) * _Pi[1] * twistExp(_ptr2abstract->active_twists[2], *(q+2) ) * *gst_0 ;
    setExponentials(q);
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *gst_0 ;
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsTCP] gst_x: %f", _gst(0,3));
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsTCP] gst_y: %f", _gst(1,3));
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsTCP] gst_z: %f", _gst(2,3));
    return;
}

Eigen::Vector3f ScrewsKinematics::updatePositionTCP(float *q) {
    // returns the TCP position only
    Eigen::Vector3f p_tcp;
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];   
    setExponentials(q);
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *gst_0 ;
    p_tcp[0] = _gst(0,3);
    p_tcp[1] = _gst(1,3);
    p_tcp[2] = _gst(2,3);
    return p_tcp;
}

Eigen::Vector3f ScrewsKinematics::updateSpatialVelocityTCP(float *q, float *dq) {
    // [20-7-24]] Returns the spatial TCP velocity only
    // Locals, ugly but 1st+easy step
    Eigen::Vector3f v_s_tcp_3;
    Eigen::Vector4f v_s_tcp;
    Eigen::Vector4f p_s_tcp;
    Eigen::Matrix<float, 6, 1> V_tcp;
    Eigen::Matrix4f V_tcp_matrix;

    // Calculate TCP tf
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];   

    setExponentials(q);
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *gst_0 ;

    ToolVelocityTwist(ScrewsKinematics::JacobianSelection::SPATIAL , dq, V_tcp);
    formTwist(V_tcp_matrix, V_tcp);

    p_s_tcp = _gst.matrix().col(3);

    v_s_tcp = V_tcp_matrix * p_s_tcp;

    v_s_tcp_3 = v_s_tcp.head<3>();

    return v_s_tcp_3;
}

Eigen::Vector3f ScrewsKinematics::updatePositionTCP(Eigen::Matrix<float, 3, 1>& q) {
    Eigen::Vector3f p_tcp;
    Eigen::Isometry3f gst_0 = *(_ptr2abstract->gsai_ptr[3]);   
    setExponentials(q.data()); // Assuming setExponentials still requires a float pointer
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * gst_0;
    p_tcp[0] = _gst(0, 3);
    p_tcp[1] = _gst(1, 3);
    p_tcp[2] = _gst(2, 3);
    return p_tcp;
}

void ScrewsKinematics::ForwardKinematicsTCP() {
    _debug_verbosity = false;
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];   
    //ROS_INFO("_joint_pos_x: %f", _joint_pos[0]);
    //ROS_INFO("_joint_pos_y: %f", _joint_pos[1]);
    //ROS_INFO("_joint_pos_z: %f", _joint_pos[2]);
    setExponentials(_joint_pos);
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *gst_0 ;
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsTCP] gst_x: %f", _gst(0,3));
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsTCP] gst_y: %f", _gst(1,3));
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsTCP] gst_z: %f", _gst(2,3));
    return;
}

void ScrewsKinematics::ForwardKinematics3DOF_1(float *q, Eigen::Isometry3f* gs_a_i[DOF+1]) {
    // Returns a matrix of pointers of the robot's active joints' frames
    // @ the given configuration.
    // "_1" -> Implements eq.8/p.44/[2]
    // Calculates the "Ci_1" matrices /in screw_dynamics/calculateFwdKinMueller.m [laptop]
    _debug_verbosity = false;

    // Calculate 1st joint frame
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[0]), _ptr2abstract->active_twists_anat[0]);
    *gs_a_i[0] = *(_ptr2abstract->gsai_test_ptr[0]) *  twistExp(_X, _joint_pos[0]);
    _trans_vector = gs_a_i[0]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_test_ptr[1]), *(_ptr2abstract->gsai_test_ptr[0]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[1]), _ptr2abstract->active_twists_anat[1]);
    *gs_a_i[1] = *gs_a_i[0] * _Bi * twistExp(_X, q[1]);
    _trans_vector = gs_a_i[1]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z()); 
    
    // Calculate 3rd joint frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_test_ptr[2]), *(_ptr2abstract->gsai_test_ptr[1]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[2]), _ptr2abstract->active_twists_anat[2]);
    *gs_a_i[2] =  *gs_a_i[1] * _Bi * twistExp(_X, q[2]);
    _trans_vector = gs_a_i[2]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_test_ptr[3]), *(_ptr2abstract->gsai_test_ptr[2]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[2]), _ptr2abstract->active_twists_anat[2]);
    *gs_a_i[3] =  *gs_a_i[2] * _Bi;  // this must be equal to _gst /in ForwardKinematicsTCP()
    _trans_vector = gs_a_i[3]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gst_x_1: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gst_y_1: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gst_z_1: %f", _trans_vector.z()); 
    
    return;
}

void ScrewsKinematics::ForwardKinematics3DOF_1() {
    // MUST CALL BEFORE USE:
    // 1. updateJointState
    // NOTES:
    // "_1" -> Implements eq.8/p.44/[2]
    // Calculates the "Ci_1" matrices /in screw_dynamics/calculateFwdKinMueller.m [laptop]
    _debug_verbosity = true;

    //printIsometryMatrix( *(_ptr2abstract->gsai_ptr[0]) );
    //printIsometryMatrix( *(_ptr2abstract->gsai_ptr[1]) );
    //printIsometryMatrix( *(_ptr2abstract->gsai_ptr[2]) );

    // Calculate 1st joint frame
    //_X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[0]), _ptr2abstract->active_twists[0]);
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[0]), _ptr2abstract->active_twists_anat[0]);
    //ROS_DEBUG_COND(_debug_verbosity, "[ForwardKinematics3DOF_1] 1_X_1: ");
    //if (_debug_verbosity) { printTwist(_X); }
    g[0] = *(_ptr2abstract->gsai_test_ptr[0]) *  twistExp(_X, _joint_pos[0]);
    _trans_vector = g[0].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //_Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[1]), *(_ptr2abstract->gsai_ptr[0]));
    //_X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[1]), _ptr2abstract->active_twists[1]);
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_test_ptr[1]), *(_ptr2abstract->gsai_test_ptr[0]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[1]), _ptr2abstract->active_twists_anat[1]);
    //ROS_DEBUG_COND(_debug_verbosity, "[ForwardKinematics3DOF_1] 2_X_2: ");
    //if (_debug_verbosity) { printTwist(_X); }
    g[1] = g[0] * _Bi * twistExp(_X, _joint_pos[1]);
    _trans_vector = g[1].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z()); 
    
    // Calculate 3rd joint frame
    //_Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[2]), *(_ptr2abstract->gsai_ptr[1]));
    //_X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[2]), _ptr2abstract->active_twists[2]);
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_test_ptr[2]), *(_ptr2abstract->gsai_test_ptr[1]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[2]), _ptr2abstract->active_twists_anat[2]);    
    //ROS_DEBUG_COND(_debug_verbosity, "[ForwardKinematics3DOF_1] 3_X_3: ");
    //if (_debug_verbosity) { printTwist(_X); }    
    g[2] =  g[1] * _Bi * twistExp(_X, _joint_pos[2]);
    _trans_vector = g[2].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    //_Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[3]), *(_ptr2abstract->gsai_ptr[2]));
    //_X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[2]), _ptr2abstract->active_twists[2]);
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_test_ptr[3]), *(_ptr2abstract->gsai_test_ptr[2]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_test_ptr[2]), _ptr2abstract->active_twists_anat[2]);
    //ROS_DEBUG_COND(_debug_verbosity, "[ForwardKinematics3DOF_1] 4_X_4: ");
    //if (_debug_verbosity) { printTwist(_X); }
    g[3] =  g[2] * _Bi;  // this must be equal to _gst /in ForwardKinematicsTCP()
    _trans_vector = g[3].translation();
    _gst = g[3];

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
    setExponentials(q);
    // Calculate 1st joint frame
    //*gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    *gs_a_i[0] = _active_expos[0] * *(_ptr2abstract->gsai_ptr[0]) ;
    _trans_vector = gs_a_i[0]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //*gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    *gs_a_i[1] = _active_expos[0] * _Pi[0] * _active_expos[1] * *(_ptr2abstract->gsai_ptr[1]) ;
    _trans_vector = gs_a_i[1]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    //*gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    *gs_a_i[2] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] *_active_expos[2] * *(_ptr2abstract->gsai_ptr[2]) ;    
    _trans_vector = gs_a_i[2]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    //*gs_a_i[3] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[3]) ;
    *gs_a_i[3] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *(_ptr2abstract->gsai_ptr[3]) ;    
    _trans_vector = gs_a_i[3]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gst_x_2: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gst_y_2: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gst_z_2: %f", _trans_vector.z()); 
    
    return;
}

void ScrewsKinematics::ForwardKinematics3DOF_2() {
    // MUST CALL BEFORE USE:
    // 1. updateJointState
    // NOTES:
    // "_2" -> Implements eq.94/p.241/[3], "this is also the classic Murray Book equation"
    // "_2" -> Only this updated the pointer, to access value from inherited classes
    
    _debug_verbosity = true;
    setExponentials(_joint_pos);
    // Calculate 1st joint frame
    //*gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    g[0] = _active_expos[0] * *(_ptr2abstract->gsai_ptr[0]) ;
    _trans_vector = g[0].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //*gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    g[1] = _active_expos[0] * _Pi[0] * _active_expos[1] * *(_ptr2abstract->gsai_ptr[1]) ;
    _trans_vector = g[1].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    //*gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    g[2] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] *_active_expos[2] * *(_ptr2abstract->gsai_ptr[2]) ;    
    _trans_vector = g[2].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gs3_z: %f", _trans_vector.z()); 
    
    // Calculate {T} frame
    //*gs_a_i[3] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[3]) ;
    g[3] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *(_ptr2abstract->gsai_ptr[3]) ;    
    _trans_vector = g[3].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gst_x_2: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gst_y_2: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematics3DOF_2] gst_z_2: %f", _trans_vector.z()); 
    
    g_ptr[0] = &g[0];
    g_ptr[1] = &g[1];
    g_ptr[2] = &g[2];
    g_ptr[3] = &g[3];
    _gst = g[3];

    if (_debug_verbosity) {
        std::cout << "[ForwardKinematics3DOF_2] gs_a1:\n" << g[0].matrix() << std::endl;
        std::cout << "[ForwardKinematics3DOF_2] gs_a2:\n" << g[1].matrix() << std::endl;
        std::cout << "[ForwardKinematics3DOF_2] gs_a3:\n" << g[2].matrix() << std::endl;
        std::cout << "[ForwardKinematics3DOF_2] gs_t:\n"  << g[3].matrix() << std::endl;
        std::cout << "[ForwardKinematics3DOF_2] Pi_0:\n"  << _Pi[0].matrix() << std::endl;
        std::cout << "[ForwardKinematics3DOF_2] Pi_1:\n"  << _Pi[1].matrix() << std::endl;
    }

    return;
}



void ScrewsKinematics::ForwardKinematicsComFrames3DOF_2(float *q, Eigen::Isometry3f* gs_l_i[DOF]) {
    // Returns a matrix of pointers of the robot's link COM frames
    // @ the given configuration. input is the pointer to active joints'
    // frame @ zero config
    // "_2" -> Implements eq.94/p.241/[3], "this is also the classic Murray Book equation"
    _debug_verbosity = false;
    setExponentialsAnat(q);
    // Calculate 1st joint frame
    //*gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    *gs_l_i[0] = _active_expos_anat[0] * *(_ptr2abstract->gsli_test_ptr[0]) ;
    _trans_vector = gs_l_i[0]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //*gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    // *gs_l_i[1] = _active_expos[0] * _Pi[0] * _active_expos[1] * *(_ptr2abstract->gsai_ptr[1]) ; // [11-7-24] Discontinued because active twists will be prior transformed from pseudojoints (xi_ai_anat from MATLAB)
    *gs_l_i[1] = _active_expos_anat[0] * _active_expos_anat[1] * *(_ptr2abstract->gsli_test_ptr[1]) ;
    _trans_vector = gs_l_i[1]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    //*gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    //*gs_l_i[2] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] *_active_expos[2] * *(_ptr2abstract->gsai_ptr[2]) ; // [11-7-24] Discontinued because active twists will be prior transformed from pseudojoints (xi_ai_anat from MATLAB)   
    *gs_l_i[2] = _active_expos_anat[0] * _active_expos_anat[1] * _active_expos_anat[2] * *(_ptr2abstract->gsli_test_ptr[2]) ;
    _trans_vector = gs_l_i[2]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl3_z: %f", _trans_vector.z()); 

    return;
}

void ScrewsKinematics::ForwardKinematicsComFrames3DOF_2() {
    // MUST CALL BEFORE USE:
    // 1. updateJointState
    // NOTES:
    // "_2" -> Implements eq.94/p.241/[3], "this is also the classic Murray Book equation"
    // "_2" -> Only this updated the pointer, to access value from inherited classes
    _debug_verbosity = false;
    setExponentialsAnat(_joint_pos);
    // Calculate 1st joint frame
    //*gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    gl[0] = _active_expos_anat[0] * *(_ptr2abstract->gsli_test_ptr[0]) ;
    _trans_vector = gl[0].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //*gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    //gl[1] = _active_expos[0] * _Pi[0] * _active_expos[1] * *(_ptr2abstract->gsai_ptr[1]) ; // [11-7-24] Discontinued because active twists will be prior transformed from pseudojoints (xi_ai_anat from MATLAB)
    gl[1] = _active_expos_anat[0] * _active_expos_anat[1] * *(_ptr2abstract->gsli_test_ptr[1]) ; 
    _trans_vector = gl[1].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    //*gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    //gl[2] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] *_active_expos[2] * *(_ptr2abstract->gsai_ptr[2]) ;    // [11-7-24] Discontinued because active twists will be prior transformed from pseudojoints (xi_ai_anat from MATLAB)
    gl[2] = _active_expos_anat[0] * _active_expos_anat[1] * _active_expos_anat[2] * *(_ptr2abstract->gsli_test_ptr[2]) ;    
    
    _trans_vector = gl[2].translation();
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"[ForwardKinematicsComFrames3DOF_2] gsl3_z: %f", _trans_vector.z()); 

    ptr2_gl[0] = &g[0];
    ptr2_gl[1] = &g[1];
    ptr2_gl[2] = &g[2];

    return;
}

void ScrewsKinematics::SpatialJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jsp_t_1[DOF]) {
    // Executes first "=" of eq.28/p.52/[2]
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. The local screw coord vectors must be initialized!
    //          3. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = false;

    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[i]);
        *Jsp_t_1[i] = _ad * iXi[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Spatial Jacobian 1: "); print6nMatrix(Jsp_t_1, DOF);}
}

void ScrewsKinematics::SpatialJacobian_Tool_1() {
    _debug_verbosity = false;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[i]);
        *ptr2Jsp1[i] = _ad * iXi[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Spatial Jacobian 1: "); print6nMatrix(ptr2Jsp1, DOF);}
}

void ScrewsKinematics::SpatialJacobian_Tool_2(Eigen::Matrix<float, 6, 1> *Jsp_t_2[DOF]) {
    // Executes second "=" of eq.28/p.52/[2]
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = true;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[i] * _ptr2abstract->gsai_ptr[i]->inverse());
        *Jsp_t_2[i] = _ad * _ptr2abstract->active_twists[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Spatial Jacobian 2: "); print6nMatrix(Jsp_t_2, DOF);}
}

void ScrewsKinematics::SpatialJacobian_Tool_2() {
    _debug_verbosity = true;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[i] * _ptr2abstract->gsai_ptr[i]->inverse());
        *ptr2Jsp2[i] = _ad * _ptr2abstract->active_twists[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Spatial Jacobian 2: "); print6nMatrix(ptr2Jsp2, DOF);}
}

void ScrewsKinematics::BodyJacobians(Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[DOF+1]) {
    // [25-7-24] Implements functions: calculateBodyJacobians1_Mueller located in:
    //           ~/matlab_ws/screw_dynamics/calculateBodyJacobians1_Mueller.m @ laptop-WIN10
    // Executes first "=" of eq.16/p.49/[2], for ALL active joints' frames && {T} frame

    _debug_verbosity = true;
    Eigen::Matrix<float, 6, 1>* Jbd_i[DOF]; // array of pointers to access a single(!) 6x3 Body Jacobian
    // Allocate the memory for the above pointers!

    for (size_t nFrames = 0; nFrames < DOF+1; nFrames++)
    {
        for (size_t i = 0; i < DOF; i++) {
            Jbd_i[i] = new Eigen::Matrix<float, 6, 1>;
        }
        for (size_t i = 0; i < DOF; i++){
            ad(_ad, g[nFrames].inverse() * g[i] );
            *Jbd_i[i] = _ad * iXi[i];
        }    
        if (_debug_verbosity) {  ROS_INFO("Body Frame Jacobian : "); print6nMatrix(Jbd_i, DOF);}
        for (size_t i = 0; i < DOF; i++) {
            *BodyJacobiansFrames[nFrames][i] = *Jbd_i[i];
        }
    }
    return;
}

void ScrewsKinematics::BodyJacobians() {
    // [25-7-24] Implements functions: calculateBodyJacobians1_Mueller located in:
    //           ~/matlab_ws/screw_dynamics/calculateBodyJacobians1_Mueller.m @ laptop-WIN10
    // Executes first "=" of eq.16/p.49/[2], for ALL active joints' frames && {T} frame
    _debug_verbosity = true;
    Eigen::Matrix<float, 6, 1>* Jbd_i[DOF]; // array of pointers to access a single(!) 6x3 Body Jacobian
    // Allocate the memory for the above pointers!

    for (size_t nFrames = 0; nFrames < DOF+1; nFrames++)
    {
        for (size_t i = 0; i < DOF; i++) {
            Jbd_i[i] = new Eigen::Matrix<float, 6, 1>;
        }
        for (size_t i = 0; i < DOF; i++){
            ad(_ad, g[nFrames].inverse() * g[i] );
            *Jbd_i[i] = _ad * iXi[i];
        }    
        if (_debug_verbosity) {  ROS_INFO("Body Frame Jacobian : "); print6nMatrix(Jbd_i, DOF);}
        for (size_t i = 0; i < DOF; i++) {
            *ptr2BodyJacobiansFrames[nFrames][i] = *Jbd_i[i];
        }
    }
    return;
}

void ScrewsKinematics::BodyCOMJacobians() {
    // [25-7-24] Implements functions: calculateLinkCoMJacobians_3DoF located in:
    //           ~/matlab_ws/screw_dynamics/calculateLinkCoMJacobians_3DoF.m @ laptop-WIN10

    _debug_verbosity = true;

    // _active_expos_anat[i]
    setExponentialsAnat(_joint_pos); // since gsli is used, the anat twists must be used!: 
    
    // Assuming _active_expos_anat is an array of Eigen::Isometry3f
    if (_active_expos_anat == nullptr) {
        std::cerr << "_active_expos_anat is not initialized!" << std::endl;
        return;
    }

    // Ensure gsli_test_ptr is not null
    for (int i = 0; i < DOF; ++i) {
        if (_ptr2abstract->gsli_test_ptr[i] == nullptr) {
            std::cerr << "gsli_test_ptr[" << i << "] is null!" << std::endl;
            return;
        }
    }

    // Ensure active_twists_anat is not null
    for (int i = 0; i < DOF; ++i) {
        if (_ptr2abstract->active_twists_anat[i].size() == 0) {
            std::cerr << "active_twists_anat[" << i << "] is not initialized!" << std::endl;
            return;
        }
    }

    // Fill Jbsli by col: [Eigen::Matrix<float, 6, 1> Jbsli[DOF][DOF];]
    for (int m = 0; m < DOF; ++m) {
        try {
            if (m==0) {
                Jbsli[m][0] = ad(_active_expos_anat[0] * *(_ptr2abstract->gsli_test_ptr[0]) ).inverse() * _ptr2abstract->active_twists_anat[0];
                Jbsli[m][1] = Eigen::Matrix<float, 6, 1>::Zero();
                Jbsli[m][2] = Eigen::Matrix<float, 6, 1>::Zero();
            } else if (m==1) {
                Jbsli[m][0] = ad(_active_expos_anat[0] * _active_expos_anat[1] * *(_ptr2abstract->gsli_test_ptr[1]) ).inverse() * _ptr2abstract->active_twists_anat[0];
                Jbsli[m][1] = ad(_active_expos_anat[1] * *(_ptr2abstract->gsli_test_ptr[1]) ).inverse() * _ptr2abstract->active_twists_anat[1];
                Jbsli[m][2] = Eigen::Matrix<float, 6, 1>::Zero();
            } else if (m==2) {
                Jbsli[m][0] = ad(_active_expos_anat[0] * _active_expos_anat[1] * _active_expos_anat[2] * *(_ptr2abstract->gsli_test_ptr[2]) ).inverse() * _ptr2abstract->active_twists_anat[0];
                Jbsli[m][1] = ad(_active_expos_anat[1] * _active_expos_anat[2] * *(_ptr2abstract->gsli_test_ptr[2]) ).inverse() * _ptr2abstract->active_twists_anat[1];
                Jbsli[m][2] = ad(_active_expos_anat[2] * *(_ptr2abstract->gsli_test_ptr[2]) ).inverse() * _ptr2abstract->active_twists_anat[2];
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception in calculating Jbsli[" << m << "]: " << e.what() << std::endl;
            return;
        }
    }

    // Link the pointers to Jbsli: [Eigen::Matrix<float, 6, 1>* ptr_Jbsli[DOF][DOF];]
    for (int m = 0; m < DOF; ++m) {
        for (int n = 0; n < DOF; ++n) {
            ptr_Jbsli[m][n] = &Jbsli[m][n];
            //*ptr_Jbsli[m][n] = someFunctionToFillCol(); // Fill the 6x1 matrix
        }
    }
    
    for (int m = 0; m < DOF; ++m) {
        for (int n = 0; n < DOF; ++n) {
            Jbsli63[m].col(n) = Jbsli[m][n];      // Fill the a6x3 arrays: [Eigen::Matrix<float, 6, DOF> Jbsli63[DOF];]
            //ptr_Jbsli63[m]->col(n) = Jbsli[m][n]; // Link the pointers to 6x3 arrays: [Eigen::Matrix<float, 6, DOF>* ptr_Jbsli63[DOF];]
        }
    }
    
    if (_debug_verbosity) {print63MatrixByColumn(Jbsli63);}

    return;
}

void ScrewsKinematics::BodyJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jbd_t_1[DOF]) {
    // Executes first "=" of eq.16/p.49/[2] BUT only for the {T} frame
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. The local screw coord vectors must be initialized!
    //          3. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = true;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[DOF].inverse() * g[i] );
        *Jbd_t_1[i] = _ad * iXi[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Body Jacobian Tool 1: "); print6nMatrix(Jbd_t_1, DOF);}
}

void ScrewsKinematics::BodyJacobian_Tool_1() {
    // Executes first "=" of eq.16/p.49/[2] BUT only for the {T} frame
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. The local screw coord vectors must be initialized!
    //          3. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = false;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[DOF].inverse() * g[i] );
        *ptr2Jbd_t_1[i] = _ad * iXi[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Body Jacobian Tool 1: "); print6nMatrix(ptr2Jbd_t_1, DOF);}
}

void ScrewsKinematics::BodyJacobian_Tool_2(Eigen::Matrix<float, 6, 1> *Jbd_t_2[DOF]) {
    // Executes second "=" of eq.16/p.49/[2]  BUT only for the {T} frame
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. The local screw coord vectors must be initialized!
    //          3. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = true;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[DOF].inverse() * g[i] * _ptr2abstract->gsai_ptr[i]->inverse() );
        *Jbd_t_2[i] = _ad * _ptr2abstract->active_twists[i] ;
    }
    if (_debug_verbosity) {  ROS_INFO("Body Jacobian Tool 2: "); print6nMatrix(Jbd_t_2, DOF);}
}

void ScrewsKinematics::BodyJacobian_Tool_2() {
    // Executes second "=" of eq.16/p.49/[2]  BUT only for the {T} frame
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. The local screw coord vectors must be initialized!
    //          3. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = true;
    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[DOF].inverse() * g[i] * _ptr2abstract->gsai_ptr[i]->inverse() );
        *ptr2Jbd_t_2[i] = _ad * _ptr2abstract->active_twists[i] ;
    }
    if (_debug_verbosity) {  ROS_INFO("Body Jacobian Tool 2: "); print6nMatrix(ptr2Jbd_t_2, DOF);}
}

void ScrewsKinematics::ToolVelocityTwist(typ_jacobian jacob_selection, float *dq, Eigen::Matrix<float, 6, 1> &Vtwist ) {
    // Returns the spatial OR the body velocity twist @ current [q,dq]
    // The twist returned relates to the Jacobian Matrix specified using "Jacob_select"
    _debug_verbosity = false;

    // Form vector from joint velocities for proper multiplication
    Eigen::Vector3f dq_vector;
    dq_vector << dq[0], dq[1], dq[2];

    switch (jacob_selection)
    {
    case typ_jacobian::SPATIAL :
        // Concatenate the Spatial Jacobian column vectors to a single array:
        Jsp63 = mergeColumns2Matrix63(Jsp_t_1);    
        Vtwist = Jsp63 * dq_vector;
        if (_debug_verbosity) {  ROS_INFO("Spatial Velocity Twist: "); printTwist( Vtwist);}
        break;
    case typ_jacobian::BODY :
        // Concatenate the Spatial Jacobian column vectors to a single array:
        Jbd63 = mergeColumns2Matrix63(Jbd_t_1);    
        Vtwist = Jbd63 * dq_vector;    
        if (_debug_verbosity) {  ROS_INFO("Body Velocity Twist: "); printTwist( Vtwist);}
        break;    
    default:
        ROS_ERROR("WRONG JACOBIAN SELECTION FOR VELOCITY TWIST");
        break;
    }
    return; 
}

void ScrewsKinematics::ToolVelocityTwist(typ_jacobian jacob_selection) {
    // Returns the spatial OR the body velocity twist @ current [q,dq]
    // The twist returned relates to the Jacobian Matrix specified using "Jacob_select"
    _debug_verbosity = false;

    // Form vector from joint velocities for proper multiplication
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];

    switch (jacob_selection)
    {
    case typ_jacobian::SPATIAL :
        // Concatenate the Spatial Jacobian column vectors to a single array:
        Jsp63 = mergeColumns2Matrix63(Jsp_t_1);    
        Vsp_tool_twist = Jsp63 * dq_vector;
        if (_debug_verbosity) {  ROS_INFO("Spatial Velocity Twist: "); printTwist(Vsp_tool_twist);}
        break;
    case typ_jacobian::BODY :
        // Concatenate the Spatial Jacobian column vectors to a single array:
        Jbd63 = mergeColumns2Matrix63(Jbd_t_1);    
        Vbd_tool_twist = Jbd63 * dq_vector;    
        if (_debug_verbosity) {  ROS_INFO("Body Velocity Twist: "); printTwist( Vbd_tool_twist);}
        break;    
    default:
        ROS_ERROR("WRONG JACOBIAN SELECTION FOR VELOCITY TWIST");
        break;
    }
    return; 
}

void ScrewsKinematics::DtSpatialJacobian_Tool_1( float *dq, Eigen::Matrix<float, 6, 1> *Jsp_t_1[DOF], Eigen::Matrix<float, 6, 1> *dJsp_t_1[DOF] ) {
    // Implements the first "=" in eq.(17)/p.223/[3]
    Eigen::Matrix<float, 6, 1> dJ;
    for (size_t j = 0; j < DOF; j++)
    {
        for (size_t k = 0; k < j; k++)
        {
            dJ = dJ + lb(*Jsp_t_1[k], *Jsp_t_1[j]) * dq[k];
        }
        *dJsp_t_1[j] = dJ;
    }
    return;
}

void ScrewsKinematics::DtSpatialJacobian_Tool_1() {
    // Implements the first "=" in eq.(17)/p.223/[3]
    Eigen::Matrix<float, 6, 1> dJ;
    for (size_t j = 0; j < DOF; j++)
    {
        for (size_t k = 0; k < j; k++)
        {
            dJ = dJ + lb(*ptr2Jsp1[k], *ptr2Jsp1[j]) * _joint_vel[k];
        }
        *ptr2dJsp_t_1[j] = dJ;
    }
    return;
}

void ScrewsKinematics::DtBodyJacobian_Tool_1( float *dq, Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[DOF+1], Eigen::Matrix<float, 6, 1> *dJbd_t_1[DOF]) {
    // Implements the first "=" in eq.(8)/p.223/[3], outputs the derivative of
    // the Body Jacobian /{T}
    
    _debug_verbosity = false;
    Eigen::Matrix<float, 6, 1> dJ;
    for (size_t j = 0; j < DOF; j++)
    {
        dJ.setZero(); 
        for (size_t k = j+1; k < DOF; k++)
        {
            dJ = dJ + lb(*BodyJacobiansFrames[DOF][j], *BodyJacobiansFrames[DOF][k]) * dq[k] ;
        }
        *dJbd_t_1[j] = dJ;
    }
    if (_debug_verbosity) {  ROS_INFO("Time Derivative Body Jacobian Tool 1: "); print6nMatrix(dJbd_t_1, DOF);}
    return;
}

void ScrewsKinematics::DtBodyJacobian_Tool_1() {
    // Implements the first "=" in eq.(8)/p.223/[3], outputs the derivative of
    // the Body Jacobian /{T}
    _debug_verbosity = true;
    Eigen::Matrix<float, 6, 1> dJ;

    BodyJacobians(); // updates the ptr2BodyJacobiansFrames
    for (size_t j = 0; j < DOF; j++)
    {
        dJ.setZero(); 
        for (size_t k = j+1; k < DOF; k++)
        {
            dJ = dJ + lb(*ptr2BodyJacobiansFrames[DOF][j], *ptr2BodyJacobiansFrames[DOF][k]) * _joint_vel[k] ;
        }
        *ptr2dJbd_t_1[j] = dJ;
    }
    if (_debug_verbosity) {  ROS_INFO("Time Derivative Body Jacobian Tool 1: "); print6nMatrix(ptr2dJbd_t_1, DOF);}
    return;
}

void ScrewsKinematics::DtBodyJacobian_Tool_2( float *dq, Eigen::Matrix<float, 6, 1>** BodyJacobiansFrames[DOF+1], Eigen::Matrix<float, 6, 1> *dJbd_t_2[DOF]) {
    // Implements the second "=" in eq.(8)/p.223/[3], outputs the derivative of
    // the Body Jacobian /{T}

    _debug_verbosity = false;
    Eigen::Matrix<float, 6, 1> dJ;
    Eigen::Matrix<float, 6, 1> Adad3;
    for (size_t j = 0; j < DOF; j++)
    {
        dJ.setZero(); 
        for (size_t k = j+1; k < DOF; k++)
        {
            ad(_ad, g[DOF].inverse() * g[k]);
            spatialCrossProduct(_scp, iXi[k]);
            Adad3 = - _ad * _scp * *BodyJacobiansFrames[k][j];
            dJ = dJ + Adad3 * dq[k] ;
        }
        *dJbd_t_2[j] = dJ;
    }
    if (_debug_verbosity) {  ROS_INFO("Time Derivative Body Jacobian Tool 2: "); print6nMatrix(dJbd_t_2, DOF);}
    return;
}

void ScrewsKinematics::DtBodyJacobian_Tool_2() {
    // Implements the second "=" in eq.(8)/p.223/[3], outputs the derivative of
    // the Body Jacobian /{T}
    _debug_verbosity = false;
    Eigen::Matrix<float, 6, 1> dJ;
    Eigen::Matrix<float, 6, 1> Adad3;
    for (size_t j = 0; j < DOF; j++)
    {
        dJ.setZero(); 
        for (size_t k = j+1; k < DOF; k++)
        {
            ad(_ad, g[DOF].inverse() * g[k]);
            spatialCrossProduct(_scp, iXi[k]);
            Adad3 = - _ad * _scp * *ptr2BodyJacobiansFrames[k][j];
            dJ = dJ + Adad3 * _joint_vel[k] ;
        }
        *ptr2dJbd_t_2[j] = dJ;
    }
    if (_debug_verbosity) {  ROS_INFO("Time Derivative Body Jacobian Tool 2: "); print6nMatrix(ptr2dJbd_t_2, DOF);}
    return;
}

void ScrewsKinematics::OperationalSpaceJacobian(Eigen::Matrix3f &Jop_t) {
    // Returns the Operational Space Jacobian Matrix for 3dof serial manipulator. Links
    // cartesian velocity with joints velocities. Since 3DOF, no rotational part is co-
    // nsidered. Forward Kinematics and Body Jacobian /{T} must be previously extracted for
    // the current configuration.
    // -> sets:Eigen::Matrix3f Jop
    _debug_verbosity = false;
    setBodyPositionJacobian();
    Jop_t = g[DOF].rotation() * Jbd_pos;
    if (_debug_verbosity)
    {
        ROS_INFO("Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        Jop_t(0, 0), Jop_t(0, 1), Jop_t(0, 2),
        Jop_t(1, 0), Jop_t(1, 1), Jop_t(1, 2),
        Jop_t(2, 0), Jop_t(2, 1), Jop_t(2, 2)); 
    }
    ptr2Jop = &Jop_t;
    return; 
}

void ScrewsKinematics::OperationalSpaceJacobian() {
    // Returns the Operational Space Jacobian Matrix for 3dof serial manipulator. Links
    // cartesian velocity with joints velocities. Since 3DOF, no rotational part is co-
    // nsidered. Forward Kinematics and Body Jacobian /{T} must be previously extracted for
    // the current configuration.
    // -> sets:Eigen::Matrix3f Jop
    _debug_verbosity = true;
    setBodyPositionJacobian();
    Jop = g[DOF].rotation() * Jbd_pos;
    ptr2Jop = &Jop;
    if (_debug_verbosity)
    {
        ROS_INFO("Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        Jop(0, 0), Jop(0, 1), Jop(0, 2),
        Jop(1, 0), Jop(1, 1), Jop(1, 2),
        Jop(2, 0), Jop(2, 1), Jop(2, 2)); 
    }
    return; 
}

Eigen::Matrix3f ScrewsKinematics::OperationalSpaceJacobian(const Eigen::Vector3f& qs) {
    // [22-7-24] Implements MATLAB function given in:
    //           D:\matlab_ws\Kinematic_Model_Assembly_SMM\calculateFunctions\calculateToolJacobian_3dof.m

    Eigen::Matrix3f Jtool;

    Jtool(0, 0) = ptr2Jsp1[0]->coeff(0) - ptr2Jsp1[0]->coeff(5) * qs(1) + ptr2Jsp1[0]->coeff(4) * qs(2);
    Jtool(0, 1) = ptr2Jsp1[1]->coeff(0) - ptr2Jsp1[1]->coeff(5) * qs(1) + ptr2Jsp1[1]->coeff(4) * qs(2);
    Jtool(0, 2) = ptr2Jsp1[2]->coeff(0) - ptr2Jsp1[2]->coeff(5) * qs(1) + ptr2Jsp1[2]->coeff(4) * qs(2);

    Jtool(1, 0) = ptr2Jsp1[0]->coeff(1) + ptr2Jsp1[0]->coeff(5) * qs(0) - ptr2Jsp1[0]->coeff(3) * qs(2);
    Jtool(1, 1) = ptr2Jsp1[1]->coeff(1) + ptr2Jsp1[1]->coeff(5) * qs(0) - ptr2Jsp1[1]->coeff(3) * qs(2);
    Jtool(1, 2) = ptr2Jsp1[2]->coeff(1) + ptr2Jsp1[2]->coeff(5) * qs(0) - ptr2Jsp1[2]->coeff(3) * qs(2);

    Jtool(2, 0) = ptr2Jsp1[0]->coeff(2) - ptr2Jsp1[0]->coeff(4) * qs(0) + ptr2Jsp1[0]->coeff(3) * qs(1);
    Jtool(2, 1) = ptr2Jsp1[1]->coeff(2) - ptr2Jsp1[1]->coeff(4) * qs(0) + ptr2Jsp1[1]->coeff(3) * qs(1);
    Jtool(2, 2) = ptr2Jsp1[2]->coeff(2) - ptr2Jsp1[2]->coeff(4) * qs(0) + ptr2Jsp1[2]->coeff(3) * qs(1);
    ptr2Jop = &Jtool;
    return Jtool;
}

Eigen::Matrix3f* ScrewsKinematics::getOperationalJacobian() {
    return ptr2Jop;
}

void ScrewsKinematics::inverseOperationalSpaceJacobian() {
    // Returns the inverse of the Operational Space Jacobian Matrix for 3dof serial manipulator
    // Jop must be previously extracted

    _debug_verbosity = true;
    iJop = (*ptr2Jop).inverse();
    ptr2iJop = &iJop;
    if (_debug_verbosity)
    {
        ROS_INFO("Inverse Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        iJop(0, 0), iJop(0, 1), iJop(0, 2),
        iJop(1, 0), iJop(1, 1), iJop(1, 2),
        iJop(2, 0), iJop(2, 1), iJop(2, 2)); 
    }
    return; 
}

Eigen::Matrix3f* ScrewsKinematics::getInverseOperationalJacobian() {
    return ptr2iJop;
}

Eigen::Matrix3f ScrewsKinematics::OperationalSpaceJacobian2() {
    // [22-7-24] Implements TCP Jacobian calculation used in:
    //           D:\matlab_ws\ros-gazebo-simulation\smm_ros_gazebo_quintic_optimization.m
    //           Only applies to 3DOF robot!

    // Extract the rotation part of the _gst matrix
    Eigen::Matrix3f  gst_rot = _gst.rotation();
    // Extract the top-left 3x3 submatrix of ptr2Jbd_t_2[3]
    Eigen::Matrix3f Jb_mu2;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Jb_mu2(i, j) = ptr2Jbd_t_2[3]->coeff(i, j);
        }
    }
    // Perform the matrix multiplication
    Eigen::Matrix3f J = gst_rot * Jb_mu2;
    ptr2Jop = &J;
    return J;
}

float ScrewsKinematics::KinematicManipulabilityIndex(const Eigen::Matrix3f& J) {
    return J.determinant();
}

float ScrewsKinematics::KinematicManipulabilityIndex() {
    return ptr2Jop->determinant();
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> ScrewsKinematics::KinematicManipulabilityEllipsoid(const Eigen::Matrix3f& J) {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Vector3f singular_values = svd.singularValues();
    return std::make_pair(U, singular_values);
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> ScrewsKinematics::KinematicManipulabilityEllipsoid() {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(*ptr2Jop, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Vector3f singular_values = svd.singularValues();
    return std::make_pair(U, singular_values);
}

void ScrewsKinematics::DtOperationalSpaceJacobian(Eigen::Matrix3f &dJop_t) {
    // Returns the Time DerivativeOperational Space Jacobian Matrix.
    // Needs the spatial velocity twist, the FK tf of {T} frame and
    // Body Jacobian and its first time derivative
    // -> sets:Eigen::Matrix3f dJop
    //_debug_verbosity = false; 
    setBodyPositionJacobian(); // -> Jbd_pos
    setDtBodyPositionJacobian(); // -> dJbd_pos
    setDtRotationMatrix(); // -> dRst
    dJop_t = (dRst *  Jbd_pos) + ( g[DOF].rotation() * dJbd_pos );
    ptr2dJop = &dJop_t;
    //if (_debug_verbosity)
    //{
    //    ROS_INFO("Time Derivative of Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
    //    dJop_t(0, 0), dJop_t(0, 1), dJop_t(0, 2),
    //    dJop_t(1, 0), dJop_t(1, 1), dJop_t(1, 2),
    //    dJop_t(2, 0), dJop_t(2, 1), dJop_t(2, 2)); 
    //}
    return; 
}

void ScrewsKinematics::DtOperationalSpaceJacobian() {
    // Returns the Time DerivativeOperational Space Jacobian Matrix.
    // Needs the spatial velocity twist, the FK tf of {T} frame and
    // Body Jacobian and its first time derivative
    // -> sets:Eigen::Matrix3f dJop
    _debug_verbosity = true; 

    setBodyPositionJacobian(); // -> Jbd_pos
    setDtBodyPositionJacobian(); // -> dJbd_pos
    setDtRotationMatrix(); // -> dRst
    
    dJop = (dRst *  Jbd_pos) + ( g[DOF].rotation() * dJbd_pos );
    ptr2dJop = &dJop;
    if (_debug_verbosity)
    {
        ROS_INFO("Time Derivative of Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        dJop(0, 0), dJop(0, 1), dJop(0, 2),
        dJop(1, 0), dJop(1, 1), dJop(1, 2),
        dJop(2, 0), dJop(2, 1), dJop(2, 2)); 
    }
    return; 
}

Eigen::Matrix3f* ScrewsKinematics::getDerivativeOperationalJacobian() {
    return ptr2dJop;
}

void ScrewsKinematics::DtToolVelocityTwist(typ_jacobian jacob_selection) {
    // Returns the First Time Derivative of Velocity Twist (~Acceleration twist) of the {T} frame
    // Spatial and Body Jacobians must be be previously extracted for the current configuration.
    _debug_verbosity = false;

    // Form vector from joint velocities/accelerations for proper multiplication
    Eigen::Vector3f dq_vector;
    Eigen::Vector3f ddq_vector;
    Eigen::Matrix<float, 6, 1> dV1;
    Eigen::Matrix<float, 6, 1> dV2;

    switch (jacob_selection)
    {
    case typ_jacobian::SPATIAL :
        dV1.setZero();
        dV2.setZero();
        for (size_t j = 0; j < DOF; j++)
        {
            dV1 = dV1 + Jsp_t_1[j] * _joint_accel[j];
        }
        for (size_t k = 0; k < DOF; k++)
        {
            for (size_t j = k+1; j < DOF; j++)
            {
                dV2 = dV2 + lb(Jsp_t_1[k], Jsp_t_1[j]) * _joint_vel[j] * _joint_vel[k];
            }
        }
        dVsp_tool_twist = dV1 + dV2;
        if (_debug_verbosity) {  ROS_INFO(" [ScrewsKinematics/DtToolVelocityTwist] Spatial Acceleration Twist: "); printTwist(dVsp_tool_twist);}
        break;
    case typ_jacobian::BODY :
        dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
        ddq_vector << _joint_accel[0], _joint_accel[1], _joint_accel[2];
        Jbd63 = mergeColumns2Matrix63(Jbd_t_1);
        dJbd63 = mergeColumns2Matrix63(dJbd_t_1);  
        dVbd_tool_twist = Jbd63 * ddq_vector + dJbd63 * dq_vector;
        if (_debug_verbosity) {  ROS_INFO("Body Acceleration Twist: "); printTwist(dVbd_tool_twist);}
        break;    
    default:
        ROS_ERROR("[ScrewsKinematics/DtToolVelocityTwist] WRONG JACOBIAN SELECTION FOR ACCELEARATION TWIST");
        break;
    }    
    return;
}

void ScrewsKinematics::DtToolVelocityTwist(typ_jacobian jacob_selection, float *ddq, float *dq, Eigen::Matrix<float, 6, 1> &dVtwist ) {
    // Returns the First Time Derivative of Velocity Twist (~Acceleration twist) of the {T} frame
    // Spatial and Body Jacobians must be be previously extracted for the current configuration.
    _debug_verbosity = false;

    // Form vector from joint velocities/accelerations for proper multiplication
    Eigen::Vector3f dq_vector;
    Eigen::Vector3f ddq_vector;
    Eigen::Matrix<float, 6, 1> dV1;
    Eigen::Matrix<float, 6, 1> dV2;

    switch (jacob_selection)
    {
    case typ_jacobian::SPATIAL :
        dV1.setZero();
        dV2.setZero();
        for (size_t j = 0; j < DOF; j++)
        {
            dV1 = dV1 + Jsp_t_1[j] * ddq[j];
        }
        for (size_t k = 0; k < DOF; k++)
        {
            for (size_t j = k+1; j < DOF; j++)
            {
                dV2 = dV2 + lb(Jsp_t_1[k], Jsp_t_1[j]) * dq[j] * dq[k];
            }
        }
        dVtwist = dV1 + dV2;
        if (_debug_verbosity) {  ROS_INFO("Spatial Acceleration Twist: "); printTwist(dVtwist);}
        break;
    case typ_jacobian::BODY :
        dq_vector << dq[0], dq[1], dq[2];
        ddq_vector << ddq[0], ddq[1], ddq[2];
        Jbd63 = mergeColumns2Matrix63(Jbd_t_1);
        dJbd63 = mergeColumns2Matrix63(dJbd_t_1);  
        dVtwist = Jbd63 * ddq_vector + dJbd63 * dq_vector;
        if (_debug_verbosity) {  ROS_INFO("Body Acceleration Twist: "); printTwist(dVtwist);}
        break;    
    default:
        ROS_ERROR("WRONG JACOBIAN SELECTION FOR ACCELEARATION TWIST");
        break;
    }    
    return;
}

void ScrewsKinematics::CartesianVelocity_twist(Eigen::Vector4f &v_qs) {
    // extracts the cartesian velocity by the Spatial Velocity twist
    // Spatial velocity twisat and forward kinematics tf of {T} at 
    // current configuration must be previously extracted.
    _debug_verbosity = false;
    formTwist(_twist_se3, Vsp_tool_twist);
    Eigen::Vector4f vector4f;
    vector4f << g[3](0, 3), g[3](1, 3), g[3](2, 3), 1.0;
    v_qs = _twist_se3 * vector4f;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianVelocity_jacob(Eigen::Vector3f &v_qs) {
    // Operational Space Jacobian must have been previously extracted
    _debug_verbosity = false;
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    v_qs = Jop * dq_vector;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianVelocity_jacob(Eigen::Vector3f &v_qs, Eigen::Matrix3f Jop_loc) {
    // Operational Space Jacobian is passed to the function
    // Is used when the matrix can be retrieved from another
    // server in the ROS system
    _debug_verbosity = false;
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    v_qs = Jop_loc * dq_vector;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianVelocity_jacob(Eigen::Vector4f &v_qs) {
    // Operational Space Jacobian must have been previously extracted
    _debug_verbosity = false;
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    Eigen::Vector3f v_qs3 = Jop * dq_vector;
    v_qs << v_qs3, 1.0f;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianAcceleration_twist(Eigen::Vector4f &a_qs, Eigen::Vector4f v_qs ) {
    _debug_verbosity = false;
    Eigen::Matrix4f dV_twist_matrix;
    Eigen::Matrix4f V_twist_matrix;
    formTwist(dV_twist_matrix, dVsp_tool_twist);
    Eigen::Vector4f vector4f;
    vector4f << g[3](0, 3), g[3](1, 3), g[3](2, 3), 1.0;
    formTwist(V_twist_matrix, Vsp_tool_twist);
    a_qs = dV_twist_matrix * vector4f + V_twist_matrix * v_qs;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Acceleration: [%f, %f, %f]", a_qs[0], a_qs[1], a_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianAcceleration_jacob(Eigen::Vector3f &a_qs) {
    // Operational Space Jacobians (&Derivative) must be 
    // previously extracted through functions:
    // 1. OperationalSpaceJacobian
    // 2. DtOperationalSpaceJacobian
    _debug_verbosity = false;
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    Eigen::Vector3f ddq_vector;
    ddq_vector << _joint_accel[0], _joint_accel[1], _joint_accel[2];
    DtOperationalSpaceJacobian(dJop);
    a_qs = Jop * ddq_vector + dJop * dq_vector;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Acceleration: [%f, %f, %f]", a_qs[0], a_qs[1], a_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianAcceleration_jacob(Eigen::Vector4f &a_qs) {
    // Operational Space Jacobians (&Derivative) must be 
    // previously extracted through functions:
    // 1. OperationalSpaceJacobian
    // 2. DtOperationalSpaceJacobian
    // Handles a 4f vector for acceleration
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    Eigen::Vector3f ddq_vector;
    ddq_vector << _joint_accel[0], _joint_accel[1], _joint_accel[2];
    DtOperationalSpaceJacobian(dJop);
    Eigen::Vector3f a_qs3 = Jop * ddq_vector + dJop * dq_vector;
    a_qs << a_qs3, 1.0f;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Acceleration: [%f, %f, %f]", a_qs[0], a_qs[1], a_qs[2]);}
    return;
}



/*
 *  SET FUNCTIONS
 */
void ScrewsKinematics::setExponentials(float *q) {
    // [14-7-24] Calculates the active joints exponentials
    //           Uses the reference anatomy active twists,
    //           MUST be used in POE expressions WITH pseudo tfs

    for (size_t i = 0; i < DOF; i++)
    {
        _active_expos[i] = twistExp(_ptr2abstract->active_twists[i], *(q+i) );
    }
    return;
}

void ScrewsKinematics::setExponentialsAnat(float *q) {
    // [14-7-24] Calculates the active joints exponentials
    //           Uses the test anatomy active twists,
    //           NO pseudo tfs should be included in the
    //           POE expressions.
    //           Active twists for test anatomy must be calculated!
    for (size_t i = 0; i < DOF; i++)
    {
        _active_expos_anat[i] = twistExp(_ptr2abstract->active_twists_anat[i], *(q+i) );
    }
    return;
}

void ScrewsKinematics::setBodyPositionJacobian() {
    // Sets the position part of the Tool Body jacobian,
    // as a Eigen::Matrix3f type.
    _debug_verbosity = true;

    Eigen::Matrix<float, 3, 1> Jpos_col[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        Jpos_col[i] = ptr2Jbd_t_1[i]->block<3, 1>(0, 0);
    }
    Jbd_pos << Jpos_col[0], Jpos_col[1], Jpos_col[2];  

    if (_debug_verbosity) {
        ROS_INFO("[setBodyPositionJacobian]");
        for (int i = 0; i < DOF; ++i) {
            std::cout << "Jbd_pos[" << i << "]:\n" << Jpos_col[i] << std::endl;
        }
    }

    return;   
}

void ScrewsKinematics::setDtBodyPositionJacobian() {
    // Sets the position part of the Time Derivative of 
    // Tool Body jacobian, as a Eigen::Matrix3f type.
    _debug_verbosity = true;

    Eigen::Matrix<float, 3, 1> dJpos_col[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        dJpos_col[i] = ptr2dJbd_t_1[i]->block<3, 1>(0, 0);
        //dJpos_col[i] = dJbd_t_1[i].block<3, 1>(0, 0);
    }
    dJbd_pos << dJpos_col[0], dJpos_col[1], dJpos_col[2];  

    if (_debug_verbosity) {
        ROS_INFO("[setDtBodyPositionJacobian]");
        for (int i = 0; i < DOF; ++i) {
            std::cout << "dJpos_col[" << i << "]:\n" << dJpos_col[i] << std::endl;
        }
    }

    return;   
}

void ScrewsKinematics::setDtRotationMatrix() {
    // Spatial Velocity Twist and FK tf of {T} frame at current configuration
    _debug_verbosity = true;
    
    Eigen::Matrix3f Rst = g[DOF].rotation(); 
    Eigen::Vector3f w_st_s = Vsp_tool_twist.block(3, 0, 3, 1); 
    dRst = skew(w_st_s) * Rst.transpose().inverse();
    if (_debug_verbosity) { 
        ROS_INFO("[setDtRotationMatrix]");
        std::cout << "dRst:\n" << dRst << std::endl;
    }
    return;
}
/*
 *  PRINTING FUNCTIONS-USED FOR DEBUGGING
 */
void ScrewsKinematics::printIsometryMatrix(const Eigen::Isometry3f& matrix) {
    for (int i = 0; i < 4; ++i) { ROS_INFO("%.4f\t%.4f\t%.4f\t%.4f", matrix(i, 0), matrix(i, 1), matrix(i, 2), matrix(i, 3)); }
    return;
}

void ScrewsKinematics::print6nMatrix(Eigen::Matrix<float, 6, 1>* matrices[], const int n) {
    // This prints columns (easy but ungly)
    /*
    for (size_t j = 0; j < columns; j++) {
        std::cout << *(matrices[j]) << std::endl;  
    }
    */
    // This prints elements of the vectors, row by row (beauty)
    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < n; j++) {
            std::cout << (*matrices[j])(i, 0) << "\t";
        }
        std::cout << std::endl;
    }    
    return;
}

void ScrewsKinematics::printTwist(Eigen::Matrix<float, 6, 1> Twist) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << Twist[i] << std::endl;
    }
    return;
}

void ScrewsKinematics::print63MatrixByColumn(const Eigen::Matrix<float, 6, DOF> J63[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        std::cout << "J63[" << i << "]:" << std::endl;
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < DOF; ++col) {
                std::cout << J63[i](row, col) << "\t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}