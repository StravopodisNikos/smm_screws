#include "smm_screws/ScrewsKinematics.h"

ScrewsKinematics::ScrewsKinematics() {};

ScrewsKinematics::ScrewsKinematics(const std::shared_ptr<RobotAbstractBase>& robot_def)
    : _robot_def(robot_def) {
    
    _total_pseudojoints = _robot_def->get_STRUCTURE_ID();
    std::cout << "pseudos=" << _total_pseudojoints << std::endl;
    
    _meta1_pseudojoints = _robot_def->get_PSEUDOS_METALINK1();
    _meta2_pseudojoints = _robot_def->get_PSEUDOS_METALINK2();
}

void ScrewsKinematics::extractPseudoTfs() {
// Executed during object creation, initializes the fixed transformations induces by anatomy metamorphosis
// [5-10-23] Current version, ONLY supports 3dof robot, with 2 metamorphic links.
    if (_total_pseudojoints == 0)
    {
        for (size_t i = 0; i < METALINKS; i++) {
            _Pi[i] = Eigen::Isometry3f::Identity(); }
    } else if (_total_pseudojoints == 2) {
        for (size_t i = 0; i < METALINKS; i++)
        {
            if (i == 0) // 1st metamorphic link
            {
                _last_twist_cnt = 0;
                _last_expo = Eigen::Isometry3f::Identity();
                for (size_t j = 0; j < _meta1_pseudojoints; j++)
                {
                    _Pi[i] = _last_expo * twistExp(_robot_def->get_PASSIVE_TWISTS(_last_twist_cnt), _robot_def->get_PSEUDO_ANGLES(_last_twist_cnt));
                    _last_expo = _Pi[i];
                    _last_twist_cnt++;
                }
            } else if (i == 1) // 2nd metamorphic link
            {
                _last_expo = Eigen::Isometry3f::Identity();
                for (size_t j = 0; j < _meta2_pseudojoints; j++)
                {
                    _Pi[i] = _last_expo * twistExp(_robot_def->get_PASSIVE_TWISTS(_last_twist_cnt), _robot_def->get_PSEUDO_ANGLES(_last_twist_cnt));
                    _last_expo = _Pi[i];
                    _last_twist_cnt++;
                }            
            }  
        }
    }
    std::cout << "Pi0 = " << _Pi[0].matrix() << std::endl;
    std::cout << "Pi1 = " << _Pi[1].matrix() << std::endl;
}

Eigen::Isometry3f ScrewsKinematics::ForwardKinematicsTCP(float *q) {
    Eigen::Isometry3f g;
    g = *(_robot_def->gsai_ptr[3]);
    std::cout << g.matrix() << std::endl;

    return twistExp(_robot_def->active_twists[0], *(q+0) ) * _Pi[0] * twistExp(_robot_def->active_twists[1], *(q+1) ) * _Pi[1] * twistExp(_robot_def->active_twists[2], *(q+2) ) * g ;
}