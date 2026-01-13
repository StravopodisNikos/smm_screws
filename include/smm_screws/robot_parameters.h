#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

// HERE ONLY PARAMS THAT ARE ALWAYS VALID FOR 
// - SMM ROBOT STRUCTURES
// - FIXED PARAMS USED IN LOOPS/ALGEBRA etc.
// COMMENTS:
// 1. SET CONSTANTS FOR N DOF SYNTHESIS! [10-1-2026]

namespace robot_params {
    // 3 DOF ONLY
    static constexpr int DOF                = 3; // only for 3dof
    static constexpr int METALINKS          = 2; // only for 3dof
    // 3-6 DOF [N DOF]
    static constexpr int MIN_DOF                = 3;
    static constexpr int MIN_METALINKS          = 2;
    static constexpr int MAX_DOF                = 6; 
    static constexpr int MAX_METALINKS          = 3;
}

namespace algebra_params {
    static constexpr int MATRIX3F_SIZE      = 9;
    static constexpr int MATRIX4F_SIZE      = 16;
    static constexpr int TWIST6F_SIZE       = 6;
    static constexpr int MATRIX6x6F_SIZE    = 36;
}

#endif  // ROBOT_PARAMETERS_H
