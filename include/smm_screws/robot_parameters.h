#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

// HERE ONLY PARAMS THAT ARE ALWAYS VALID FOR 
// - SMM ROBOT STRUCTURES
// - FIXED PARAMS USED IN LOOPS/ALGEBRA etc.

namespace robot_params {
    static constexpr int DOF                = 3;
    static constexpr int METALINKS          = 2;
}

namespace algebra_params {
    static constexpr int MATRIX3F_SIZE      = 9;
    static constexpr int MATRIX4F_SIZE      = 16;
    static constexpr int TWIST6F_SIZE       = 6;
    static constexpr int MATRIX6x6F_SIZE    = 36;
}

#endif  // ROBOT_PARAMETERS_H
