#include "smm_screws/ScrewsDynamics.h"

ScrewsDynamics::ScrewsDynamics() {};

ScrewsDynamics::ScrewsDynamics(RobotAbstractBase *ptr2abstract):  _ptr2abstract(ptr2abstract) {
    _total_pseudojoints = _ptr2abstract->get_STRUCTURE_ID();
    _meta1_pseudojoints = _ptr2abstract->get_PSEUDOS_METALINK1();
    _meta2_pseudojoints = _ptr2abstract->get_PSEUDOS_METALINK2();
    _last_twist_cnt =0;
    _last_expo = Eigen::Isometry3f::Identity();
    _debug_verbosity = true;
    _Mib[0].setIdentity();
    _Mib[1].setIdentity();
    _Mib[2].setIdentity();
    _Mis[0].setIdentity();
    _Mis[1].setIdentity();
    _Mis[2].setIdentity();    
    _alpha_temp.setZero();
    _ad_temp.setZero();
    _Ml_temp.setZero();

    // Preallocate memory for active exponential matrices used in internally calculations
    for (size_t i = 0; i < DOF; i++)
    {
        exp_ai[i] = Eigen::Isometry3f::Identity();
        ptr2active_expos[i] = &exp_ai[i];
    }    
    for (size_t i = 0; i < DOF+1; i++)
    {
        gai[i] = Eigen::Isometry3f::Identity();
        ptr2active_tfs[i] = &gai[i];
    }    
    // Preallocate memory for passive exponential matrices used in internally calculations
    for (size_t i = 0; i < METALINKS; i++)
    {
        gpj[i] = Eigen::Isometry3f::Identity();
        ptr2passive_tfs[i] = &gpj[i]; // Now the gpj's can be used from ScrewsDynamics methods
    }
    // Initialize Joint positions to zero (will be erased)
    for (size_t i = 0; i < DOF; i++) {
        _joint_pos[i] = 0;
        _joint_vel[i] = 0;
    }
    _PotEnergy_prev = 0; // dummy initialization
}

void ScrewsDynamics::updateJointPos(float *q_new) {
    // Updates current position and stores Delta Position for numeric diff.
    for (size_t i = 0; i < DOF; i++) {
        _joint_pos_prev[i] = _joint_pos[i]; // Save current position to previous
        _joint_pos[i] = q_new[i];           // Update current pos
        _delta_joint_pos[i] = _joint_pos[i] - _joint_pos_prev[i];
        //std::cout << "New Joint Posistion:" << _joint_pos[i] << std::endl;
    } 
    return;
}

void ScrewsDynamics::updateJointVel(float *dq_new) {
    for (size_t i = 0; i < DOF; i++) {_joint_vel[i] = dq_new[i];}
    return;
}

void ScrewsDynamics::intializeLinkMassMatrices() {
    // Loads the Links' Mass matrices @ {S} Frame
    _Mis[0] = *(_ptr2abstract->Mi_s_ptr[0]);
    _Mis[1] = *(_ptr2abstract->Mi_s_ptr[1]);
    _Mis[2] = *(_ptr2abstract->Mi_s_ptr[2]);

        return;
}

Eigen::Matrix3f ScrewsDynamics::MassMatrix() {
    // Calculates the Mass Matrix 
    _debug_verbosity = false;

    //std::cout << "Joint Posistion1:" << _joint_pos[0] << std::endl;
    //std::cout << "Joint Posistion1:" << _joint_pos[1] << std::endl;
    //std::cout << "Joint Posistion1:" << _joint_pos[2] << std::endl;
    ScrewsDynamics::updateActiveExpos();
    /*
    printIsometryMatrix(gai[0]);
    printIsometryMatrix(gai[1]);
    printIsometryMatrix(gai[2]);
    
    printIsometryMatrix(*ptr2active_tfs[0]);
    printIsometryMatrix(*ptr2active_tfs[1]);
    printIsometryMatrix(*ptr2active_tfs[2]);
    */
    size_t max;
    MM.setZero();
    if (_debug_verbosity) {std::cout << "[MassMatrix] M: " << "\n";}
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            max = (i > j) ? i : j; // assigns the max to l
            for (size_t l = max; l < DOF; l++)
            {
                //std::cout << l << std::endl;
                _alpha[0] = setAlphamatrix(l, i); //print66Matrix(_alpha[0]); // -> ok  // Ali
                _alpha[1] = setAlphamatrix(l, j); //print66Matrix(_alpha[1]); // -> ok  // Alj
                //print66Matrix(_Mib[l]); // -> ok

                // [12-7-24] On 13-2-24 a huge BUG was found in MATLAB code file: compute_mij_429_3DoF_2_sym.m
                // Link Inertia Matrix is already expressed in {S} frame during assembly, so it is not needed to
                // re-execute the adjoint tf! The inertias of each link must be provided in robot_definition.h
                // based on data obtained by M_s_link_as_anat 3D array!
                //_Ml_temp = ad(((_ptr2abstract->gl[l])).inverse()).transpose() * _Mib[l] * ad(((_ptr2abstract->gl[l])).inverse()); 
                _Ml_temp = _Mis[l];
                //print66Matrix(_Ml_temp); // -> not ok
                //MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists[j];   
                MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists_anat[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists_anat[j];     
            }
            if (_debug_verbosity) {std::cout << MM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    ptr2MM = &MM;
    return MM;
}

void ScrewsDynamics::MassMatrix_loc() {
    // Calculates the Mass Matrix, used locally in class member functions
    _debug_verbosity = false;
    ScrewsDynamics::updateActiveExpos();

    size_t max;
    MM.setZero();
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            max = (i > j) ? i : j; // assigns the max to l
            for (size_t l = max; l < DOF; l++)
            {
                _alpha[0] = setAlphamatrix(l, i); // print66Matrix(_alpha[0]); // -> ok  // Ali
                _alpha[1] = setAlphamatrix(l, j); // print66Matrix(_alpha[1]); // -> ok  // Alj
                //_Ml_temp = ad(((_ptr2abstract->gl[l])).inverse()).transpose() * _Mib[l] * ad(((_ptr2abstract->gl[l])).inverse()); 
                _Ml_temp = _Mis[l];
                //MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists[j];   
                MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists_anat[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists_anat[j];               
            }
            if (_debug_verbosity) {std::cout << MM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    ptr2MM = &MM;
    return;
}

Eigen::Matrix3f ScrewsDynamics::CoriolisMatrix() {
    // Calculates the Coriolis Matrix 
    _debug_verbosity = false;
    CM.setZero();
    if (_debug_verbosity) {std::cout << "[CoriolisMatrix] C: " << "\n";}
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            for (size_t k = 0; k < DOF; k++)
            {   
                parDerMass[0](i, j) = computeParDerMassElement(i, j, k)(0,0); // delat_Mij_theta_k
                parDerMass[1](i, j) = computeParDerMassElement(i, k, j)(0,0); // delat_Mik_theta_j
                parDerMass[2](i, j) = computeParDerMassElement(k, j, i)(0,0); // delat_Mkj_theta_i
                ChristoffelSymbols[k](i, j) = 0.5 * (parDerMass[0](i, j) + parDerMass[1](i, j) - parDerMass[2](i, j));
                CM(i, j) = CM(i, j) + ( ChristoffelSymbols[k](i, j) * _joint_vel[k] ); 
            }
            if (_debug_verbosity) {std::cout << CM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    
    return CM;
}

void ScrewsDynamics::CoriolisMatrix_loc() {
    // Calculates the Coriolis Matrix, used locally in class member functions 
    _debug_verbosity = false;
    CM.setZero();
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            for (size_t k = 0; k < DOF; k++)
            {   
                parDerMass[0](i, j) = computeParDerMassElement(i, j, k)(0,0); // delat_Mij_theta_k
                parDerMass[1](i, j) = computeParDerMassElement(i, k, j)(0,0); // delat_Mik_theta_j
                parDerMass[2](i, j) = computeParDerMassElement(k, j, i)(0,0); // delat_Mkj_theta_i
                ChristoffelSymbols[k](i, j) = 0.5 * (parDerMass[0](i, j) + parDerMass[1](i, j) - parDerMass[2](i, j));
                CM(i, j) = CM(i, j) + ( ChristoffelSymbols[k](i, j) * _joint_vel[k] ); 
            }
            if (_debug_verbosity) {std::cout << CM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    return;
}

Eigen::Matrix<float, 3, 1> ScrewsDynamics::GravityVector() {
    _debug_verbosity = false;
    GV.setZero();
    _PotEnergy = computePotentialEnergy();
    //std::cout << "Potential Energy: " << _PotEnergy << std::endl;
    float DeltaPotEnergy = _PotEnergy - _PotEnergy_prev;
    _PotEnergy_prev = _PotEnergy;

    for (size_t i = 0; i < DOF; i++)
    {
        if ( std::abs( _delta_joint_pos[i]) > 0.0001f ) { 
            GV(i,0) = DeltaPotEnergy / _delta_joint_pos[i]; 
        } else { GV(i,0) = 0; }
        //if (_debug_verbosity) {std::cout << GV(i,0) << std::endl;}
    }

    return GV;
}

void ScrewsDynamics::LinkGeometricJacobians() {
    // This is MATLAB function: ~/matlab_ws/screw_dynamics/calculateLinkGeometricJacobians.m

    // 1. Build the T
    // [NEED UPGRADE] current joint tfs will be retrieved from /robot_screw_states
    // Need to call in main cpp file: ForwardKinematics3DOF_2();
    updateActiveTfs();


    // 2. Build Tl
    // [NEED UPGRADE] current joint tfs will be retrieved from /robot_screw_states
    // Need to call in main cpp file:  ForwardKinematicsComFrames3DOF_2();
    updateCOMTfs();

    //std::cout << "[LinkGeometricJacobians] gs_a1:\n" << gai[0].matrix() << std::endl;
    //std::cout << "[LinkGeometricJacobians] gs_a2:\n" << gai[1].matrix() << std::endl;
    //std::cout << "[LinkGeometricJacobians] gs_a3:\n" << gai[2].matrix() << std::endl;

    // 3. Extract the z-rot-axis vectors
    // ln.16-18 in MATLAB file. 1st axis is z, 2nd+3rd axes are x!
    Eigen::Vector3f z_01 = gai[0].matrix().block<3, 1>(0, 2); // extracts a block starting at row 0, column 2, with a size of 3 rows and 1 column
    Eigen::Vector3f z_02 = gai[1].matrix().block<3, 1>(0, 0);
    Eigen::Vector3f z_03 = gai[2].matrix().block<3, 1>(0, 0);

    // 4. Extract the points
    Eigen::Vector3f p_01 = gai[0].matrix().block<3, 1>(0, 3); //std::cout << "Vector p_01:\n" << p_01 << std::endl;
    Eigen::Vector3f p_02 = gai[1].matrix().block<3, 1>(0, 3);
    Eigen::Vector3f p_03 = gai[2].matrix().block<3, 1>(0, 3);
    Eigen::Vector3f pl_01 = gsli[0].matrix().block<3, 1>(0, 3); //std::cout << "Vector pl_01:\n" << pl_01 << std::endl;
    Eigen::Vector3f pl_02 = gsli[1].matrix().block<3, 1>(0, 3);
    Eigen::Vector3f pl_03 = gsli[2].matrix().block<3, 1>(0, 3);

    // 5. Build the columns
    Eigen::Vector3f O31 = Eigen::Vector3f::Zero();

    Eigen::Vector3f Jp1_11 = z_01.cross( (pl_01 - p_01) ); //std::cout << "Vector Jp1_11:\n" << Jp1_11 << std::endl;
    
    Eigen::Vector3f Jp1_12 = z_01.cross( (pl_02 - p_01) ); //std::cout << "Vector Jp1_12:\n" << Jp1_12 << std::endl;
    Eigen::Vector3f Jp2_12 = z_02.cross( (pl_02 - p_02) ); //std::cout << "Vector Jp2_12:\n" << Jp2_12 << std::endl;

    Eigen::Vector3f Jp1_13 = z_01.cross( (pl_03 - p_01) ); //std::cout << "Vector Jp1_13:\n" << Jp1_13 << std::endl;
    Eigen::Vector3f Jp2_13 = z_02.cross( (pl_03 - p_02) ); //std::cout << "Vector Jp2_13:\n" << Jp2_13 << std::endl;
    Eigen::Vector3f Jp3_13 = z_03.cross( (pl_03 - p_03) ); //std::cout << "Vector Jp3_13:\n" << Jp3_13 << std::endl;

    // 6. Fill the Jacobians
    // 1st Link Geometric Jacobian
    // Assign values to Jgl and ptr2Jgl
    Jgl[0][0] = Jp1_11;
    Jgl[0][1] = O31;
    Jgl[0][2] = O31;
    // Assign pointers to ptr2Jgl
    ptr2Jgl[0][0] = &Jgl[0][0];
    ptr2Jgl[0][1] = &Jgl[0][1];
    ptr2Jgl[0][2] = &Jgl[0][2];
    // 2nd Link Geometric Jacobian
    // Assign values to Jgl and ptr2Jgl
    Jgl[1][0] = Jp1_12;
    Jgl[1][1] = Jp2_12;
    Jgl[1][2] = O31;
    // Assign pointers to ptr2Jgl
    ptr2Jgl[1][0] = &Jgl[1][0];
    ptr2Jgl[1][1] = &Jgl[1][1];
    ptr2Jgl[1][2] = &Jgl[1][2];
    // 3rd Link Geometric Jacobian
    // Assign values to Jgl and ptr2Jgl
    Jgl[2][0] = Jp1_13;
    Jgl[2][1] = Jp2_13;
    Jgl[2][2] = Jp3_13;
    // Assign pointers to ptr2Jgl
    ptr2Jgl[2][0] = &Jgl[2][0];
    ptr2Jgl[2][1] = &Jgl[2][1];
    ptr2Jgl[2][2] = &Jgl[2][2];   

    return; 
}

Eigen::Matrix<float, DOF, 1> ScrewsDynamics::GravityVectorAnalytical() {
    // [11-7-24] Implements MATLAB function in laptop-WIN10: 
    // ~/matlab_ws/screw_dynamics/calculateGravityVectorAnalytical.m

    _debug_verbosity = false;
    GV.setZero();
    Eigen::Vector3f g_earth(0.0f, 0.0f, 9.80665f); // "-" removed and changed SIGNS in GV calculation
    
    LinkGeometricJacobians();

    float gv1_1,gv1_2,gv1_3;
    gv1_1 = ( *(_ptr2abstract->link_mass[0]) * g_earth.transpose() * Jgl[0][0]) ; 
    gv1_2 = ( *(_ptr2abstract->link_mass[1]) * g_earth.transpose() * Jgl[1][0]) ;
    gv1_3 = ( *(_ptr2abstract->link_mass[2]) * g_earth.transpose() * Jgl[2][0]) ;
    GV(0) = gv1_1 + gv1_2 + gv1_3;

    float gv2_1,gv2_2,gv2_3;
    gv2_1 = ( *(_ptr2abstract->link_mass[0]) * g_earth.transpose() * Jgl[0][1]) ;
    gv2_2 = ( *(_ptr2abstract->link_mass[1]) * g_earth.transpose() * Jgl[1][1]) ;
    gv2_3 = ( *(_ptr2abstract->link_mass[2]) * g_earth.transpose() * Jgl[2][1]) ;
    GV(1) = gv2_1 + gv2_2 + gv2_3;

    float gv3_1,gv3_2,gv3_3;
    gv3_1 = ( *(_ptr2abstract->link_mass[0]) * g_earth.transpose() * Jgl[0][2]) ;
    gv3_2 = ( *(_ptr2abstract->link_mass[1]) * g_earth.transpose() * Jgl[1][2]) ;
    gv3_3 = ( *(_ptr2abstract->link_mass[2]) * g_earth.transpose() * Jgl[2][2]) ;
    GV(2) = gv3_1 + gv3_2 + gv3_3;      

    if (_debug_verbosity) {std::cout << "[GravityVectorAnalytical] G: " << "\n";}
    for (size_t i = 0; i < DOF; i++) {
        if (_debug_verbosity) {std::cout << GV(i) << "\n";} }

    return GV;
}

Eigen::Matrix<float, DOF, 1> ScrewsDynamics::GravityVectorAnalyticalBody() {
    // [11-7-24] Implements MATLAB function in laptop-WIN10: 
    // ~/matlab_ws/screw_dynamics/calculateGravityVectorAnalyticalBody.m   
    _debug_verbosity = false;
    updateActiveTfs();
    updateCOMTfs();
    GV.setZero();
    Eigen::Vector3f g_earth(0.0f, 0.0f, 9.80665f); // "-" removed and changed SIGNS in GV calculation

    // Needs updated gl[] from ScrewsKinematics
    // Needs updates Jbsli from ScrewsKinematics

    GV(0) = (  ( (*ptr_Jbsli[0][0]).head<3>().transpose() * ( ptr2_gl[0]->rotation() *  ( *(_ptr2abstract->link_mass[0]) * g_earth ) ) ) ).value() ;

    GV(1) = ( ( (*ptr_Jbsli[0][1]).head<3>().transpose() * ( ptr2_gl[0]->rotation() *  ( *(_ptr2abstract->link_mass[0]) * g_earth ) ) ) \
            + ( (*ptr_Jbsli[1][1]).head<3>().transpose() * ( ptr2_gl[1]->rotation() *  ( *(_ptr2abstract->link_mass[1]) * g_earth ) ) )\
            + ( (*ptr_Jbsli[2][1]).head<3>().transpose() * ( ptr2_gl[2]->rotation() *  ( *(_ptr2abstract->link_mass[2]) * g_earth ) ) ) ).value() ;

    GV(2) = ( ( (*ptr_Jbsli[0][2]).head<3>().transpose() * ( ptr2_gl[0]->rotation() *  ( *(_ptr2abstract->link_mass[0]) * g_earth ) ) ) \
            + ( (*ptr_Jbsli[1][2]).head<3>().transpose() * ( ptr2_gl[1]->rotation() *  ( *(_ptr2abstract->link_mass[1]) * g_earth ) ) ) \
            + ( (*ptr_Jbsli[2][2]).head<3>().transpose() * ( ptr2_gl[2]->rotation() *  ( *(_ptr2abstract->link_mass[2]) * g_earth ) ) ) ).value() ;

    if (_debug_verbosity) {std::cout << "[GravityVectorAnalyticalBody] G: " << "\n";}
    for (size_t i = 0; i < DOF; i++) {
        if (_debug_verbosity) {std::cout << GV(i) << "\n";} }

    return GV;
}

void ScrewsDynamics::GravityVector_loc() {
    // [25-7-24] Function DEPRECATED - NO USE

    _debug_verbosity = false;
    GV.setZero();
    _PotEnergy = computePotentialEnergy();
    float DeltaPotEnergy = _PotEnergy - _PotEnergy_prev;
    _PotEnergy_prev = _PotEnergy;

    for (size_t i = 0; i < DOF; i++)
    {
        if ( std::abs( _delta_joint_pos[i]) > 0.0001 ) { 
            GV(i,0) = DeltaPotEnergy / _delta_joint_pos[i]; 
        } else { GV(i,0) = 0; }
        //if (_debug_verbosity) {std::cout << GV(i,0) << std::endl;}
    }

    return;
}

Eigen::Matrix<float, 3, 1> ScrewsDynamics::FrictionVector() {
    _debug_verbosity = false;
    FV.setZero();
    Eigen::Matrix<float, 3, 1> fc;
    Eigen::Matrix<float, 3, 1> fv;
    for (size_t i = 0; i < DOF; i++)
    {
        if ( std::signbit(_joint_vel[i]) ) { fc(i,0) = - *(_ptr2abstract->fc_coeffs[i]); }
        else { fc(i,0) = *(_ptr2abstract->fc_coeffs[i]); }
        fv(i,0) = *(_ptr2abstract->fv_coeffs[i]) * _joint_vel[i];
        FV(i,0) = fc(i,0) + fv(i,0);
        //if (_debug_verbosity) {std::cout << FV(i,0) << std::endl;}
    }
    
    return FV;
}

void ScrewsDynamics::FrictionVector_loc() {
    _debug_verbosity = false;
    FV.setZero();
    Eigen::Matrix<float, 3, 1> fc;
    Eigen::Matrix<float, 3, 1> fv;
    for (size_t i = 0; i < DOF; i++)
    {
        if ( std::signbit(_joint_vel[i]) ) { fc(i,0) = - *(_ptr2abstract->fc_coeffs[i]); }
        else { fc(i,0) = *(_ptr2abstract->fc_coeffs[i]); }
        fv(i,0) = *(_ptr2abstract->fv_coeffs[i]) * _joint_vel[i];
        FV(i,0) = fc(i,0) + fv(i,0);
        //if (_debug_verbosity) {std::cout << FV(i,0) << std::endl;}
    }
    return;
}

float ScrewsDynamics::DynamicManipulabilityIndex(const Eigen::Matrix3f& J, const Eigen::Matrix3f& M) {
    
    return ( J.determinant() / M.determinant() );
}

float ScrewsDynamics::DynamicManipulabilityIndex() {
        if (ptr2Jop == nullptr) {
        ROS_ERROR("[DynamicManipulabilityIndex] ptr2Jop is null!");
        return 0.0f;  // or handle the error appropriately
    }

    if (ptr2MM == nullptr) {
        ROS_ERROR("[DynamicManipulabilityIndex] ptr2MM is null!");
        return 0.0f;  // or handle the error appropriately
    }

    return ( ptr2Jop->determinant() / ptr2MM->determinant() );
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> ScrewsDynamics::DynamicManipulabilityEllipsoid() {

    if (ptr2Jop == nullptr) {
        throw std::runtime_error("[DynamicManipulabilityEllipsoid] ptr2Jop is null!");
    }

    if (ptr2MM == nullptr) {
        throw std::runtime_error("[DynamicManipulabilityEllipsoid] ptr2MM is null!");
    }

    // Form the combined matrix A = J * M^-1 * J^T
    Eigen::Matrix3f J = *ptr2Jop;
    Eigen::Matrix3f M_inv = ptr2MM->inverse();
    Eigen::Matrix3f A = J * M_inv * J.transpose();

    // Perform SVD on the combined matrix A
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A.inverse(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Vector3f singular_values = svd.singularValues();

    return std::make_pair(U, singular_values);
}

Eigen::Matrix<float, 6, 6> ScrewsDynamics::setAlphamatrix(size_t i, size_t j) {
    if (i == 0) {
        if ( j == 0) {
            _alpha_temp.setIdentity();
        } else if (j == 1) {
            _alpha_temp.setZero();
        } else if ( j == 2) {
            _alpha_temp.setZero();
        }
    } else if ( i == 1) {
        if ( j == 0) {
            _ad_temp = ad(*ptr2passive_tfs[j] * *ptr2active_expos[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if (j == 1) {
            _alpha_temp.setIdentity();
        } else if ( j == 2) {
            _alpha_temp.setZero();
        }        
    } else if ( i == 2) {
        if ( j == 0) {
            _ad_temp = ad(*ptr2passive_tfs[j] * *ptr2active_expos[j+1] * *ptr2passive_tfs[j+1] * *ptr2active_expos[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if (j == 1) {
            _ad_temp = ad(*ptr2passive_tfs[j] * *ptr2active_expos[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if ( j == 2) {
            _alpha_temp.setIdentity();
        }         
    } else {
        ROS_ERROR("WRONG COUNTER FOR ALPHA MATRIX SET.");    
    }
    
    return _alpha_temp;
}

Eigen::Matrix<float, 1, 1> ScrewsDynamics::computeParDerMassElement(size_t i, size_t j, size_t k) {
    // returns a value of a single element of the 
    // Partial Derivative of the Mass Matrix
    size_t max_ij;
    _parDer_MassIJ_ThetaK(0,0) = 0;
    max_ij = (i > j) ? i : j; // assigns the max to l
    for (size_t l = max_ij; l < DOF; l++)
    {
        _alphaParDer[0] = setAlphamatrix(k, i); // Aki
        _alphaParDer[1] = setAlphamatrix(l, k); // Alk
        _alphaParDer[2] = setAlphamatrix(l, j); // Alj
        //_LieBracketParDer[0] = lb(_alphaParDer[0]*(_ptr2abstract->active_twists[i]), _ptr2abstract->active_twists[k] );
        _LieBracketParDer[0] = lb(_alphaParDer[0]*(_ptr2abstract->active_twists_anat[i]), _ptr2abstract->active_twists_anat[k] );

        _alphaParDer[3] = setAlphamatrix(l, i);   // Ali
        _alphaParDer[4] = setAlphamatrix(k, j);   // Akj  
        //_LieBracketParDer[1] = lb(_alphaParDer[4]*(_ptr2abstract->active_twists[j]), _ptr2abstract->active_twists[k] );
        _LieBracketParDer[1] = lb(_alphaParDer[4]*(_ptr2abstract->active_twists_anat[j]), _ptr2abstract->active_twists_anat[k] );

        //_Ml_temp = ad(((_ptr2abstract->gl[l])).inverse()).transpose() * _Mib[l] * ad(((_ptr2abstract->gl[l])).inverse()); 
        _Ml_temp = _Mis[l];
        
        _parDer_MassIJ_ThetaK = _parDer_MassIJ_ThetaK + \
        ( (_LieBracketParDer[0].transpose() * _alphaParDer[1].transpose() * _Ml_temp * _alphaParDer[2] * (_ptr2abstract->active_twists_anat[j]) ) + \
        ( (_ptr2abstract->active_twists_anat[i]).transpose() * _alphaParDer[3].transpose() * _Ml_temp * _alphaParDer[1] * _LieBracketParDer[1]  ) );
    }
    return _parDer_MassIJ_ThetaK;
}

float ScrewsDynamics::computePotentialEnergy() {
    float pot_energy = 0;
    updateCOMTfs(); // updates gsli;
    for (size_t i = 0; i < DOF; i++)
    {
        pot_energy = pot_energy - ( *(_ptr2abstract->link_mass[i]) * _g_z * gsli[i].translation()(2) );
    }
    return pot_energy;
}

void ScrewsDynamics::updateCOMTfs() {
    // gpj are not updated, only initialized for each anatomy
    ScrewsDynamics::updateActiveExpos(); // updates gai[i]
    gsli[0] = *ptr2active_expos[0] * (_ptr2abstract->gl_test_0[0]) ; 
    gsli[1] = *ptr2active_expos[0] * *ptr2active_expos[1] * (_ptr2abstract->gl_test_0[1]) ;
    gsli[2] = *ptr2active_expos[0] * *ptr2active_expos[1] * *ptr2active_expos[2] * (_ptr2abstract->gl_test_0[2]) ;
    return;
}

void ScrewsDynamics::updateActiveTfs() {
    ScrewsDynamics::updateActiveExpos(); // updates gai[i]
 
    gai[0] = *ptr2active_expos[0] * (_ptr2abstract->g_test_0[0]) ; 
    gai[1] = *ptr2active_expos[0]  * *ptr2active_expos[1] * (_ptr2abstract->g_test_0[1]) ;
    gai[2] = *ptr2active_expos[0]  * *ptr2active_expos[1] *  *ptr2active_expos[2] * (_ptr2abstract->g_test_0[2]) ; 

    ptr2active_tfs[0] = &gai[0];
    ptr2active_tfs[1] = &gai[1];
    ptr2active_tfs[2] = &gai[2];
    return;
}

void ScrewsDynamics::updateActiveExpos() {
    // - joint pos update must be called. 
    // - ptr2active_tfs must point to preallocated memory 
    for (size_t i = 0; i < DOF; i++)
    {
        exp_ai[i] = twistExp(_ptr2abstract->active_twists_anat[i], _joint_pos[i]) ;
        ptr2active_expos[i] = &exp_ai[i];
    }
    return;
}

/*
 *  PRINTING FUNCTIONS-USED FOR DEBUGGING
 */

void ScrewsDynamics::print66Matrix(Eigen::Matrix<float, 6, 6> matrix) {
    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < 6; j++) {
            std::cout << matrix(i, j) << "\t";
        }
        std::cout << std::endl;
    }
    return;
}

void ScrewsDynamics::print61Matrix(Eigen::Matrix<float, 6, 1> matrix) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << matrix[i] << std::endl;
    }
    return;
}

void ScrewsDynamics::print16Matrix(Eigen::Matrix<float, 1, 6> matrix) {
    for (size_t i = 0; i < 6; i++) {
        std::cout << matrix[i] << "\t";
    }
    std::cout <<  std::endl;
    return;
}