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
}

void ScrewsKinematics::initializeRelativeTfs(Eigen::Isometry3f* Bi[DOF+1]) {
    // Initializes the matrices Bi ~ Ci_i1(0)
    _debug_verbosity = false;

    Bi[0] = _ptr2abstract->gsai_ptr[0];
    if (_debug_verbosity) { printIsometryMatrix(*Bi[0]); ROS_INFO(" ");}
    for (size_t i = 1; i < DOF+1; i++)
    {
        *Bi[i] = extractRelativeTf(*_ptr2abstract->gsai_ptr[i], *_ptr2abstract->gsai_ptr[i-1]);
        if (_debug_verbosity) { printIsometryMatrix(*Bi[i]); ROS_INFO(" ");}
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

void ScrewsKinematics::ForwardKinematicsTCP(float *q) {
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];   
    //_gst = twistExp(_ptr2abstract->active_twists[0], *(q+0) ) * _Pi[0] * twistExp(_ptr2abstract->active_twists[1], *(q+1) ) * _Pi[1] * twistExp(_ptr2abstract->active_twists[2], *(q+2) ) * *gst_0 ;
    setExponentials(q);
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *gst_0 ;
    ROS_INFO("gst_x: %f", _gst(0,3));
    ROS_INFO("gst_y: %f", _gst(1,3));
    ROS_INFO("gst_z: %f", _gst(2,3));
    return;
}

void ScrewsKinematics::ForwardKinematicsTCP() {
    Eigen::Isometry3f *gst_0 = _ptr2abstract->gsai_ptr[3];   
    setExponentials(_joint_pos);
    _gst = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *gst_0 ;
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
    _debug_verbosity = true;

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

void ScrewsKinematics::ForwardKinematics3DOF_1() {
    // MUST CALL BEFORE USE:
    // 1. updateJointState
    // NOTES:
    // "_1" -> Implements eq.8/p.44/[2]
    // Calculates the "Ci_1" matrices /in screw_dynamics/calculateFwdKinMueller.m [laptop]
    _debug_verbosity = true;

    // Calculate 1st joint frame
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[0]), _ptr2abstract->active_twists[0]);
    g[0] = *(_ptr2abstract->gsai_ptr[0]) *  twistExp(_X, _joint_pos[0]);
    _trans_vector = g[0].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[1]), *(_ptr2abstract->gsai_ptr[0]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[1]), _ptr2abstract->active_twists[1]);
    g[1] = g[0] * _Bi * twistExp(_X, _joint_pos[1]);
    _trans_vector = g[1].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z()); 
    
    // Calculate 3rd joint frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[2]), *(_ptr2abstract->gsai_ptr[1]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[2]), _ptr2abstract->active_twists[2]);
    g[2] =  g[1] * _Bi * twistExp(_X, _joint_pos[2]);
    _trans_vector = g[2].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    _Bi = extractRelativeTf(*(_ptr2abstract->gsai_ptr[3]), *(_ptr2abstract->gsai_ptr[2]));
    _X = extractLocalScrewCoordVector(*(_ptr2abstract->gsai_ptr[2]), _ptr2abstract->active_twists[2]);
    g[3] =  g[2] * _Bi;  // this must be equal to _gst /in ForwardKinematicsTCP()
    _trans_vector = g[3].translation();
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
    _debug_verbosity = true;
    setExponentials(q);
    // Calculate 1st joint frame
    //*gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    *gs_a_i[0] = _active_expos[0] * *(_ptr2abstract->gsai_ptr[0]) ;
    _trans_vector = gs_a_i[0]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //*gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    *gs_a_i[1] = _active_expos[0] * _Pi[0] * _active_expos[1] * *(_ptr2abstract->gsai_ptr[1]) ;
    _trans_vector = gs_a_i[1]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    //*gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    *gs_a_i[2] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] *_active_expos[2] * *(_ptr2abstract->gsai_ptr[2]) ;    
    _trans_vector = gs_a_i[2]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    //*gs_a_i[3] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[3]) ;
    *gs_a_i[3] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *(_ptr2abstract->gsai_ptr[3]) ;    
    _trans_vector = gs_a_i[3]->translation();
    ROS_DEBUG_COND(_debug_verbosity,"gst_x_2: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gst_y_2: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gst_z_2: %f", _trans_vector.z()); 
    
    return;
}

void ScrewsKinematics::ForwardKinematics3DOF_2() {
    // MUST CALL BEFORE USE:
    // 1. updateJointState
    // NOTES:
    // "_2" -> Implements eq.94/p.241/[3], "this is also the classic Murray Book equation"
    _debug_verbosity = true;
    setExponentials(_joint_pos);
    // Calculate 1st joint frame
    //*gs_a_i[0] = twistExp(_ptr2abstract->active_twists[0], q[0]) * *(_ptr2abstract->gsai_ptr[0]) ;  
    g[0] = _active_expos[0] * *(_ptr2abstract->gsai_ptr[0]) ;
    _trans_vector = g[0].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs1_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs1_z: %f", _trans_vector.z()); 

    // Calculate 2nd joint frame
    //*gs_a_i[1] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * *(_ptr2abstract->gsai_ptr[1]) ;
    g[1] = _active_expos[0] * _Pi[0] * _active_expos[1] * *(_ptr2abstract->gsai_ptr[1]) ;
    _trans_vector = g[1].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs2_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs2_z: %f", _trans_vector.z());    

    // Calculate 3rd joint frame
    //*gs_a_i[2] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[2]) ;
    g[2] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] *_active_expos[2] * *(_ptr2abstract->gsai_ptr[2]) ;    
    _trans_vector = g[2].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gs3_x: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_y: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gs3_z: %f", _trans_vector.z()); 

    // Calculate {T} frame
    //*gs_a_i[3] = twistExp(_ptr2abstract->active_twists[0], q[0]) * twistExp(_ptr2abstract->active_twists[1], q[1]) * twistExp(_ptr2abstract->active_twists[2], q[2]) * *(_ptr2abstract->gsai_ptr[3]) ;
    g[3] = _active_expos[0] * _Pi[0] * _active_expos[1] * _Pi[1] * _active_expos[2] * *(_ptr2abstract->gsai_ptr[3]) ;    
    _trans_vector = g[3].translation();
    ROS_DEBUG_COND(_debug_verbosity,"gst_x_2: %f", _trans_vector.x());
    ROS_DEBUG_COND(_debug_verbosity,"gst_y_2: %f", _trans_vector.y());
    ROS_DEBUG_COND(_debug_verbosity,"gst_z_2: %f", _trans_vector.z()); 
    
    return;
}

void ScrewsKinematics::SpatialJacobian_Tool_1(Eigen::Matrix<float, 6, 1> *Jsp_t_1[DOF]) {
    // Executes first "=" of eq.28/p.52/[2]
    // [USAGE]: 1. The fwd kin must be extracted for the current q before function call
    //          2. The local screw coord vectors must be initialized!
    //          3. All 2D matrices for kinematics are stored in array of pointers!
    _debug_verbosity = true;

    for (size_t i = 0; i < DOF; i++){
        ad(_ad, g[i]);
        *Jsp_t_1[i] = _ad * iXi[i];
    }
    if (_debug_verbosity) {  ROS_INFO("Spatial Jacobian 1: "); print6nMatrix(Jsp_t_1, DOF);}
}

void ScrewsKinematics::SpatialJacobian_Tool_1() {
    _debug_verbosity = true;
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
    _debug_verbosity = true;
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
    _debug_verbosity = true;

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
    _debug_verbosity = true;

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
    
    _debug_verbosity = true;
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

    _debug_verbosity = true;
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
    _debug_verbosity = true;
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
    _debug_verbosity = true;
    setBodyPositionJacobian();
    Jop_t = g[DOF].rotation() * Jbd_pos;
    if (_debug_verbosity)
    {
        ROS_INFO("Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        Jop_t(0, 0), Jop_t(0, 1), Jop_t(0, 2),
        Jop_t(1, 0), Jop_t(1, 1), Jop_t(1, 2),
        Jop_t(2, 0), Jop_t(2, 1), Jop_t(2, 2)); 
    }
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
    if (_debug_verbosity)
    {
        ROS_INFO("Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        Jop(0, 0), Jop(0, 1), Jop(0, 2),
        Jop(1, 0), Jop(1, 1), Jop(1, 2),
        Jop(2, 0), Jop(2, 1), Jop(2, 2)); 
    }
    return; 
}

void ScrewsKinematics::DtOperationalSpaceJacobian(Eigen::Matrix3f &dJop_t) {
    // Returns the Time DerivativeOperational Space Jacobian Matrix.
    // Needs the spatial velocity twist, the FK tf of {T} frame and
    // Body Jacobian and its first time derivative
    // -> sets:Eigen::Matrix3f dJop
    _debug_verbosity = true; 
    setBodyPositionJacobian(); // -> Jbd_pos
    setDtBodyPositionJacobian(); // -> dJbd_pos
    setDtRotationMatrix(); // -> dRst
    dJop_t = (dRst *  Jbd_pos) + ( g[DOF].rotation() * dJbd_pos );
    if (_debug_verbosity)
    {
        ROS_INFO("Time Derivative of Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        dJop_t(0, 0), dJop_t(0, 1), dJop_t(0, 2),
        dJop_t(1, 0), dJop_t(1, 1), dJop_t(1, 2),
        dJop_t(2, 0), dJop_t(2, 1), dJop_t(2, 2)); 
    }
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
    if (_debug_verbosity)
    {
        ROS_INFO("Time Derivative of Operational Space Jacobian: \n%f %f %f \n%f %f %f \n%f %f %f", 
        dJop(0, 0), dJop(0, 1), dJop(0, 2),
        dJop(1, 0), dJop(1, 1), dJop(1, 2),
        dJop(2, 0), dJop(2, 1), dJop(2, 2)); 
    }
    return; 
}

void ScrewsKinematics::DtToolVelocityTwist(typ_jacobian jacob_selection) {
    // Returns the First Time Derivative of Velocity Twist (~Acceleration twist) of the {T} frame
    // Spatial and Body Jacobians must be be previously extracted for the current configuration.
    _debug_verbosity = true;

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
        if (_debug_verbosity) {  ROS_INFO("Spatial Acceleration Twist: "); printTwist(dVsp_tool_twist);}
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
        ROS_ERROR("WRONG JACOBIAN SELECTION FOR ACCELEARATION TWIST");
        break;
    }    
    return;
}

void ScrewsKinematics::DtToolVelocityTwist(typ_jacobian jacob_selection, float *ddq, float *dq, Eigen::Matrix<float, 6, 1> &dVtwist ) {
    // Returns the First Time Derivative of Velocity Twist (~Acceleration twist) of the {T} frame
    // Spatial and Body Jacobians must be be previously extracted for the current configuration.
    _debug_verbosity = true;

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
    _debug_verbosity = true;
    formTwist(_twist_se3, Vsp_tool_twist);
    Eigen::Vector4f vector4f;
    vector4f << g[3](0, 3), g[3](1, 3), g[3](2, 3), 1.0;
    v_qs = _twist_se3 * vector4f;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianVelocity_jacob(Eigen::Vector3f &v_qs) {
    // Operational Space Jacobian must have been previously extracted
    _debug_verbosity = true;
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    v_qs = Jop * dq_vector;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianVelocity_jacob(Eigen::Vector4f &v_qs) {
    // Operational Space Jacobian must have been previously extracted
    _debug_verbosity = true;
    Eigen::Vector3f dq_vector;
    dq_vector << _joint_vel[0], _joint_vel[1], _joint_vel[2];
    Eigen::Vector3f v_qs3 = Jop * dq_vector;
    v_qs << v_qs3, 1.0f;
    if (_debug_verbosity) {ROS_INFO("Cartesian Spatial Velocity: [%f, %f, %f]", v_qs[0], v_qs[1], v_qs[2]);}
    return;
}

void ScrewsKinematics::CartesianAcceleration_twist(Eigen::Vector4f &a_qs, Eigen::Vector4f v_qs ) {
    _debug_verbosity = true;
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
    _debug_verbosity = true;
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
    // Calculates the active joints exponentials for
    // the configuration given
    for (size_t i = 0; i < DOF; i++)
    {
        _active_expos[i] = twistExp(_ptr2abstract->active_twists[i], *(q+i) );
    }
    return;
}

void ScrewsKinematics::setBodyPositionJacobian() {
    // Sets the position part of the Tool Body jacobian,
    // as a Eigen::Matrix3f type.
    Eigen::Matrix<float, 3, 1> Jpos_col[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        Jpos_col[i] = Jbd_t_1[i].block<3, 1>(0, 0);
    }
    Jbd_pos << Jpos_col[0], Jpos_col[1], Jpos_col[2];  
    return;   
}

void ScrewsKinematics::setDtBodyPositionJacobian() {
    // Sets the position part of the Time Derivative of 
    // Tool Body jacobian, as a Eigen::Matrix3f type.
    Eigen::Matrix<float, 3, 1> dJpos_col[DOF];
    for (size_t i = 0; i < DOF; i++)
    {
        dJpos_col[i] = dJbd_t_1[i].block<3, 1>(0, 0);
    }
    dJbd_pos << dJpos_col[0], dJpos_col[1], dJpos_col[2];  
    return;   
}

void ScrewsKinematics::setDtRotationMatrix() {
    // Spatial Velocity Twist and FK tf of {T} frame at current configuration
    _debug_verbosity = true;
    Eigen::Matrix3f Rst = g[DOF].rotation(); 
    Eigen::Vector3f w_st_s = Vsp_tool_twist.block(3, 0, 3, 1); 
    dRst = skew(w_st_s) * Rst.transpose().inverse();
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
