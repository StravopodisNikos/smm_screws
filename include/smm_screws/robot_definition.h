#ifndef ROBOT_DEFINITION_H
#define ROBOT_DEFINITION_H

/*
 *  [17-10-23]  Here the kinematic elements, of the active joints
 *  are manually assigned by provided by user, given the MATLAB
 *  data extracted for the smm structure/anatomy we currently operate.
 *  
 *  TO DO: Automatic extraction of these data by the urdf model.
 */

#define DOF 3

// Define and initialize your constants
namespace robot_definition {
    // Members for the abstract class
    float __active_twist_0[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float __active_twist_1[6] = {0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f};
    float __active_twist_2[6] = {0.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f};
    float gsa10[4][4]= {
    {1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , 1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 1.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};
    float gsa20[4][4]= {
    {1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , -1.0f , 0.0f },
    {0.0f , 1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};
    float gsa30[4][4]= {
    {1.0f , 0.0f , 0.0f , 1.0f },
    {0.0f , 0.0f , -1.0f , 0.0f },
    {0.0f , 1.0f , 1.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};
    float gst0[4][4]= {
    {1.0f , 0.0f , 0.0f , 1.5f },
    {0.0f , 0.0f , -1.0f , 0.0f },
    {0.0f , 1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};
    float gsl10[4][4]= {
    {1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , -1.0f , 0.0f },
    {0.0f , 1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};
    float gsl20[4][4]= {
    {1.0f , 0.0f , 0.0f , 0.5f },
    {0.0f , 0.0f , -1.0f , 0.0f },
    {0.0f , 1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};
    float gsl30[4][4]= {
    {1.0f , 0.0f , 0.0f , 1.25f },
    {0.0f , 0.0f , -1.0f , 0.0f },
    {0.0f , 1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
};    
    float __masses[DOF] = {0.5f, 1.0f, 1.0f};
    float __inertias[DOF] = {0.5f, 5.0f, 5.0f};
    float __fc_coeffs[DOF] = {1.0f, 1.0f, 1.0f};
    float __fv_coeffs[DOF] = {50.0f, 50.0f, 50.0f};

}
#endif  // ROBOT_DEFINITION_H
