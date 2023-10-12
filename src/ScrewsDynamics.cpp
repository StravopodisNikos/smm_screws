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
    MM.setZero();
    _alpha_temp.setZero();
    _ad_temp.setZero();
    _Ml_temp.setZero();
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
    return;
}

Eigen::Matrix3f ScrewsDynamics::MassMatrix() {
    // Calculates the Mass Matrix 
    _debug_verbosity = true;
    size_t l;
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