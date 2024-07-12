#ifndef PASSIVE_DEFINITION_H
#define PASSIVE_DEFINITION_H

/*
 *  [17-10-23]  Here the kinematic elements, of the PASSIVE joints
 *  are manually assigned by provided by user, given the MATLAB
 *  data extracted for the smm structure/anatomy we currently operate.
 *  
 *  TO DO: 1. Automatic extraction of these data by the urdf model.
 *         2. Add conditions given the yaml file that defines the structure string
 */

#define METALINKS 2 // available: 2,3,4 (for 3DoF SMM)

// Define and initialize your constants
namespace passive_definition {

    // Members for the derived classes 
    float __pseudo_angles[METALINKS] = {0, 0}; // depends on METALINKS definition
    float __passive_twist_0[6] = {-0.0f, 0.1660f, -0.025f, 1.0f, 0.0f, 0.0f};
    float __passive_twist_1[6] = {-0.4685f, 0.0f, -0.025f, 0.0f, -1.0f, 0.0f};
    //float __passive_twist_2[6] = ...
    //const float __passive_twist_3[6] = ...
    int __META1_PSEUDOS = 1; // available: 1,2 (each metamorphic link must have 1 pseudojoint, and max 2 )
    int __META2_PSEUDOS = 1; // available: 1,2
    
}


#endif  // PASSIVE_DEFINITION_H