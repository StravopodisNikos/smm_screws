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
    
    _alpha_temp.setZero();
    _ad_temp.setZero();
    _Ml_temp.setZero();

    // Preallocate memory for active exponential matrices used in internally calculations
    for (size_t i = 0; i < DOF; i++)
    {
        gai[i] = Eigen::Isometry3f::Identity();
        ptr2active_tfs[i] = &gai[i]; // Now the gai's can be used from ScrewsDynamics methods
    }    
    // Preallocate memory for passive exponential matrices used in internally calculations
    for (size_t i = 0; i < METALINKS; i++)
    {
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
    // Constructs the Links' Mass matrices @ Links Body (COM) Frame
    
    for (size_t i = 0; i < DOF; i++)
    {
        _Mib[i].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[i]) * Eigen::Matrix3f::Identity();
        _Mib[i].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[i].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[i].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[i]) * Eigen::Matrix3f::Identity();
    }
   
   /*
    // Added extreme to test code.
        _Mib[0].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[0]) * Eigen::Matrix3f::Identity();
        _Mib[0](0,1) = 0.05; _Mib[0](0,2) = 0.009; _Mib[0](1,0) = _Mib[0](0,1);
        _Mib[0](1,2) = 0.04; _Mib[0](2,0) = _Mib[0](0,2); _Mib[0](2,1) = _Mib[0](1,2);  
        _Mib[0].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[0].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[0].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[0]) * Eigen::Matrix3f::Identity();    
        //print66Matrix(_Mib[0]);
        _Mib[1].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[1]) * Eigen::Matrix3f::Identity();
        _Mib[1](0,1) = 0.02; _Mib[1](0,2) = 0.023; _Mib[1](1,0) = _Mib[1](0,1);
        _Mib[1](1,2) = 0.01; _Mib[1](2,0) = _Mib[1](0,2); _Mib[1](2,1) = _Mib[1](1,2);  
        _Mib[1].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[1].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[1].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[1]) * Eigen::Matrix3f::Identity();    
        //print66Matrix(_Mib[1]);
        _Mib[2].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[2]) * Eigen::Matrix3f::Identity();
        _Mib[2](0,1) = 0.034; _Mib[2](0,2) = 0.01; _Mib[2](1,0) = _Mib[2](0,1);
        _Mib[2](1,2) = 0.04; _Mib[2](2,0) = _Mib[2](0,2); _Mib[2](2,1) = _Mib[2](1,2);  
        _Mib[2].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[2].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[2].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[2]) * Eigen::Matrix3f::Identity();    
        //print66Matrix(_Mib[2]);                  
         */
        return;
}

Eigen::Matrix3f ScrewsDynamics::MassMatrix() {
    // Calculates the Mass Matrix 
    _debug_verbosity = false;

    //std::cout << "Joint Posistion1:" << _joint_pos[0] << std::endl;
    //std::cout << "Joint Posistion1:" << _joint_pos[1] << std::endl;
    //std::cout << "Joint Posistion1:" << _joint_pos[2] << std::endl;
    ScrewsDynamics::extractActiveTfs();
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
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            max = (i > j) ? i : j; // assigns the max to l
            for (size_t l = max; l < DOF; l++)
            {
                //std::cout << l << std::endl;
                _alpha[0] = setAlphamatrix(l, i); // print66Matrix(_alpha[0]); // -> ok  // Ali
                _alpha[1] = setAlphamatrix(l, j); // print66Matrix(_alpha[1]); // -> ok  // Alj
                //print66Matrix(_Mib[l]); // -> ok
                _Ml_temp = ad(((_ptr2abstract->gsli_ptr[l])).inverse()).transpose() * _Mib[l] * ad(((_ptr2abstract->gsli_ptr[l])).inverse()); 
                //print66Matrix(_Ml_temp); // -> not ok
                MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists[j];   
            }
            if (_debug_verbosity) {std::cout << MM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    
    return MM;
}

void ScrewsDynamics::MassMatrix_loc() {
    // Calculates the Mass Matrix, used locally in class member functions
    _debug_verbosity = false;
    ScrewsDynamics::extractActiveTfs();

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
                _Ml_temp = ad(((_ptr2abstract->gsli_ptr[l])).inverse()).transpose() * _Mib[l] * ad(((_ptr2abstract->gsli_ptr[l])).inverse()); 
                MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists[j];   
            }
            if (_debug_verbosity) {std::cout << MM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    
    return;
}

Eigen::Matrix3f ScrewsDynamics::CoriolisMatrix() {
    // Calculates the Coriolis Matrix 
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

void ScrewsDynamics::GravityVector_loc() {
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
            _ad_temp = ad(*ptr2passive_tfs[j] * *ptr2active_tfs[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if (j == 1) {
            _alpha_temp.setIdentity();
        } else if ( j == 2) {
            _alpha_temp.setZero();
        }        
    } else if ( i == 2) {
        if ( j == 0) {
            _ad_temp = ad(*ptr2passive_tfs[j] * *ptr2active_tfs[j+1] * *ptr2passive_tfs[j+1] * *ptr2active_tfs[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if (j == 1) {
            _ad_temp = ad(*ptr2passive_tfs[j] * *ptr2active_tfs[i]);
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
        _LieBracketParDer[0] = lb(_alphaParDer[0]*(_ptr2abstract->active_twists[i]), _ptr2abstract->active_twists[k] );

        _alphaParDer[3] = setAlphamatrix(l, i);   // Ali
        _alphaParDer[4] = setAlphamatrix(k, j);   // Akj  
        _LieBracketParDer[1] = lb(_alphaParDer[4]*(_ptr2abstract->active_twists[j]), _ptr2abstract->active_twists[k] );

        _Ml_temp = ad(((_ptr2abstract->gsli_ptr[l])).inverse()).transpose() * _Mib[l] * ad(((_ptr2abstract->gsli_ptr[l])).inverse()); 
        
        _parDer_MassIJ_ThetaK = _parDer_MassIJ_ThetaK + \
        ( (_LieBracketParDer[0].transpose() * _alphaParDer[1].transpose() * _Ml_temp * _alphaParDer[2] * (_ptr2abstract->active_twists[j]) ) + \
        ( (_ptr2abstract->active_twists[i]).transpose() * _alphaParDer[3].transpose() * _Ml_temp * _alphaParDer[1] * _LieBracketParDer[1]  ) );
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
    ScrewsDynamics::extractActiveTfs(); // updates gai[i]
    gsli[0] = *ptr2active_tfs[0] * (_ptr2abstract->gsli_ptr[0]) ; 
    gsli[1] = *ptr2active_tfs[0] * *ptr2passive_tfs[0] * *ptr2active_tfs[1] * (_ptr2abstract->gsli_ptr[1]) ;
    gsli[2] = *ptr2active_tfs[0] * *ptr2passive_tfs[0] * *ptr2active_tfs[1] * *ptr2passive_tfs[1] * *ptr2active_tfs[2] * (_ptr2abstract->gsli_ptr[2]) ; 
    return;
}

void ScrewsDynamics::extractActiveTfs() {
    // - joint pos update must be called. 
    // - ptr2active_tfs must point to preallocated memory 
    for (size_t i = 0; i < DOF; i++)
    {
        //print61Matrix(_ptr2abstract->active_twists[i]);
        *ptr2active_tfs[i] = twistExp(_ptr2abstract->active_twists[i], _joint_pos[i]) ;
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