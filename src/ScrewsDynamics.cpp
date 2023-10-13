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
}

void ScrewsDynamics::intializeLinkMassMatrices() {
    // Constructs the Links' Mass matrices @ Links Body (COM) Frame
    /*
    for (size_t i = 0; i < DOF; i++)
    {
        _Mib[i].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[i]) * Eigen::Matrix3f::Identity();
        _Mib[i].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[i].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[i].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[i]) * Eigen::Matrix3f::Identity();
    }
    */
        _Mib[0].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[0]) * Eigen::Matrix3f::Identity();
        _Mib[0](0,1) = 0.05; _Mib[0](0,2) = -0.009; _Mib[0](1,0) = -_Mib[0](0,1);
        _Mib[0](1,2) = 0.04; _Mib[0](2,0) = -_Mib[0](0,2); _Mib[0](2,1) = -_Mib[0](1,2);  
        _Mib[0].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[0].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[0].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[0]) * Eigen::Matrix3f::Identity();    
        
        _Mib[1].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[1]) * Eigen::Matrix3f::Identity();
        _Mib[1](0,1) = 0.02; _Mib[1](0,2) = -0.023; _Mib[1](1,0) = -_Mib[1](0,1);
        _Mib[1](1,2) = 0.01; _Mib[1](2,0) = -_Mib[1](0,2); _Mib[1](2,1) = -_Mib[1](1,2);  
        _Mib[1].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[1].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[1].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[0]) * Eigen::Matrix3f::Identity();    

        _Mib[2].block<3, 3>(0, 0) = *(_ptr2abstract->link_mass[2]) * Eigen::Matrix3f::Identity();
        _Mib[2](0,1) = 0.034; _Mib[2](0,2) = 0.01; _Mib[2](1,0) = -_Mib[2](0,1);
        _Mib[2](1,2) = -0.04; _Mib[2](2,0) = -_Mib[2](0,2); _Mib[2](2,1) = -_Mib[2](1,2);  
        _Mib[2].block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
        _Mib[2].block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();  
        _Mib[2].block<3, 3>(3, 3) = *(_ptr2abstract->link_inertia[2]) * Eigen::Matrix3f::Identity();    
                        
        return;
}

Eigen::Matrix3f ScrewsDynamics::MassMatrix() {
    // Calculates the Mass Matrix 
    _debug_verbosity = true;
    size_t l;
    MM.setZero();
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            l = (i > j) ? i : j; // assigns the max to l
            for (size_t add = l; add < DOF; add++)
            {
                _alpha[0] = setAlphamatrix(add, i);
                _alpha[1] = setAlphamatrix(add, j); 
                _Ml_temp = ad((*(_ptr2abstract->gsli_ptr[add])).inverse()).transpose() * _Mib[add] * ad((*(_ptr2abstract->gsli_ptr[add])).inverse()); 
                MM(i, j) = MM(i, j) + (_ptr2abstract->active_twists[i]).transpose() * _alpha[0].transpose() * _Ml_temp * _alpha[1] * _ptr2abstract->active_twists[j];   
            }
            if (_debug_verbosity) {std::cout << MM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    
    return MM;
}

Eigen::Matrix3f ScrewsDynamics::CoriolisMatrix(float *dq) {
    // Calculates the Mass Matrix 
    _debug_verbosity = true;
    CM.setZero();
    for (size_t i = 0; i < DOF; i++)
    {
        for (size_t j = 0; j < DOF; j++)
        {
            for (size_t add = 1; add < DOF; add++)
            {   
                parDerMass[0](i, j) = computeParDerMassElement(i, j, add)(0,0); // delat_Mij_theta_k
                parDerMass[1](i, j) = computeParDerMassElement(i, add, j)(0,0); // delat_Mik_theta_j
                parDerMass[2](i, j) = computeParDerMassElement(add, j, i)(0,0); // delat_Mkj_theta_i
                ChristoffelSymbols[add](i, j) = 0.5 * (parDerMass[0](i, j) + parDerMass[1](i, j) - parDerMass[2](i, j));
                CM(i, j) = CM(i, j) + ( ChristoffelSymbols[add](i, j) * dq[add] ); 
            }
            if (_debug_verbosity) {std::cout << CM(i, j) << "\t";}
        } 
        if (_debug_verbosity) {std::cout << std::endl;}
    }
    
    return CM;
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
            _ad_temp = ad(gpj[j] * gai[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if (j == 1) {
            _alpha_temp.setIdentity();
        } else if ( j == 2) {
            _alpha_temp.setZero();
        }        
    } else if ( i == 2) {
        if ( j == 0) {
            _ad_temp = ad(gpj[j] * gai[j+1] * gpj[j+1] * gai[i]);
            _alpha_temp = _ad_temp.inverse();
        } else if (j == 1) {
            _ad_temp = ad(gpj[j] * gai[i]);
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
    size_t l;
    _parDer_MassIJ_ThetaK(0,0) = 0;
    l = (i > j) ? i : j; // assigns the max to l
    for (size_t add = l; add < DOF; add++)
    {
        _alphaParDer[0] = setAlphamatrix(k, i);   // Aki
        _alphaParDer[1] = setAlphamatrix(add, k); // Alk
        _alphaParDer[2] = setAlphamatrix(add, j); // Alj
        _LieBracketParDer[0] = lb(_alphaParDer[0]*(_ptr2abstract->active_twists[i]), _ptr2abstract->active_twists[k] );

        _alphaParDer[3] = setAlphamatrix(add, i);   // Ali
        _alphaParDer[4] = setAlphamatrix(k, j);     // Akj  
        _LieBracketParDer[1] = lb(_alphaParDer[4]*(_ptr2abstract->active_twists[j]), _ptr2abstract->active_twists[k] );

        _Ml_temp = ad((*(_ptr2abstract->gsli_ptr[add])).inverse()).transpose() * _Mib[add] * ad((*(_ptr2abstract->gsli_ptr[add])).inverse()); 
        
        _parDer_MassIJ_ThetaK = _parDer_MassIJ_ThetaK + \
        ( (_LieBracketParDer[0].transpose() * _alphaParDer[1].transpose() * _Ml_temp * _alphaParDer[2] * (_ptr2abstract->active_twists[j]) ) + \
        ( (_ptr2abstract->active_twists[i]).transpose() * _alphaParDer[3].transpose() * _Ml_temp * _alphaParDer[1] * _LieBracketParDer[1]  ) );
    }
    return _parDer_MassIJ_ThetaK;
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