#ifndef ROBOT_DEFINITION_H
#define ROBOT_DEFINITION_H

/*
 *  [17-10-23]  Here the kinematic elements, of the active joints
 *  are manually assigned by provided by user, given the MATLAB
 *  data extracted for the smm structure/anatomy we currently operate.
 *  
 *  The members of the abstract class (RobotAbstractBase.h) are initialized here.
 *  
 *  TO DO: Automatic extraction of these data by the urdf model.
 */

#define DOF 3

namespace robot_definition {
    // ====== [I] Reference  anatomy ======
    // [I.1] MATLAB variable: [xi_ai_ref]
    float __active_twist_0[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float __active_twist_1[6] = {0.0f, -0.2980f, 0.0250f, -1.0f, 0.0f, 0.0f};
    float __active_twist_2[6] = {0.0f, -0.6379f, -0.1850f, -1.0f, 0.0f, 0.0f};
    
    // [I.2] MATLAB variable: [g_ai_ref]
    float gsa10[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , -1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 1.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    float gsa20[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 1.0f , 0.0250f },
    {0.0f , 1.0f , 0.0f , 0.2980f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    float gsa30[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0250f },
    {0.0f , -1.0f , 0.0f , -0.1850f },
    {0.0f , 0.0f , 1.0f , 0.6379f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    // [I.3] MATLAB variable: [gst0]
    float gst0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.025f },
    {0.0f , 0.0f , -1.0f , -0.3650f },
    {0.0f , -1.0f , -0.0f , 0.6429f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    }; 
    // ====== [II] Test  anatomy ======
    // [II.1] MATLAB variable: [xi_ai_ref]
    float __active_twist_anat_0[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float __active_twist_anat_1[6] = {0.0f, -0.2849f, -0.0323f, -1.0f, 0.0f, 0.0f};
    float __active_twist_anat_2[6] = {0.0f, -0.4384f, -0.3073f, -1.0f, 0.0f, 0.0f};

    // [II.2] MATLAB variable: [g_ai_anat]
    float gsa1_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , -1.0f , 0.0f , 0.0f },
    {0.0f , 0.0f , 1.0f , 0.0f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    float gsa2_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0f },
    {0.0f , -0.4339f , 0.9010f , -0.0323f },
    {0.0f , 0.9010f , 0.4339f , 0.2849f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    float gsa3_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0250f },
    {0.0f , -0.4339f , -0.9010f , -0.3073f },
    {0.0f , -0.9010f , 0.4339f , 0.4384f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    // [II.3] MATLAB variable: [gst_anat]
    float gst_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.025f },
    {0.0f , 0.9010f , -0.4339f , -0.3899f },
    {0.0f , -0.4339f , -0.9010f , 0.2784f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    // [II.4] MATLAB variable: [gsli0_test]
    float gsl1_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0003f },
    {0.0f , -1.0f , 0.0f , 0.0035f },
    {0.0f , 0.0f , 1.0f , 0.1488f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    float gsl2_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0154f },
    {0.0f , -0.4339f , 0.9010f , -0.1733f },
    {0.0f , 0.9010f , 0.4339f , 0.4807f },
    {0.0f , 0.0f , 0.0f , 1.0f } 
    };
    float gsl3_test_0[4][4]= {
    {-1.0f , 0.0f , 0.0f , 0.0131f },
    {0.0f , -0.4339f , -0.9010f , -0.3533f },
    {0.0f , -0.9010f , 0.4339f , 0.3485f },
    {0.0f , 0.0f , 0.0f , 1.0f }    
    };  

    // [II.5] MATLAB variable: [M_s_link_as_anat]
    float M_s_1[6][6] = {
        { 4.0407,    0.0000,   -0.0000,    0.0000,    0.6014,   -0.0140  },
        { 0.0000,    4.0407,    0.0000,   -0.6014,    0.0000,    0.0010  },
        { -0.0000,    0.0000,    4.0407,    0.0140,   -0.0010,    0.0000 },
        { 0.0000,   -0.6014,    0.0140,    0.1373,    0.0000,   -0.0000  },
        { 0.6014,    0.0000,   -0.0010,    0.0000,    0.1192,   -0.0026  },
        { -0.0140,    0.0010,    0.0000,   -0.0000,   -0.0026,    0.0193 }
    };
    float M_s_2[6][6] = {
        { 6.2914,   -0.0000,    0.0000,    0.0000,    3.0245,    1.0901  },
        {-0.0000,    6.2914,   -0.0000,   -3.0245,    0.0000,    0.0967  },
        { 0.0000,   -0.0000,    6.2914,   -1.0901,   -0.0967,    0.0000 },
        { 0.0000,   -3.0245,   -1.0901,    1.7455,    0.0222,   -0.0520 },
        { 3.0245,    0.0000,   -0.0967,    0.0222,    1.5242,    0.5559  },
        { 1.0901,    0.0967,   -0.0000,   -0.0520,    0.5559,    0.2641 }
    };
    float M_s_3[6][6] = {
        { 0.4290,   -0.0000,    0.0000 ,   0.0000 ,   0.1495 ,   0.1516  },
        { -0.0000,    0.4290,   -0.0000,   -0.1495,    0.0000,    0.0056 },
        { 0.0000 ,  -0.0000 ,   0.4290 ,  -0.1516 ,  -0.0056 ,        0 },
        { 0.0000 ,  -0.1495 ,  -0.1516 ,   0.1099 ,   0.0023 ,  -0.0013},
        { 0.1495 ,        0 ,  -0.0056 ,  0.0023  ,  0.0556  ,  0.0514},
        { 0.1516,    0.0056 ,         0,   -0.0013,    0.0514,    0.0550}
    };

    float __masses[DOF] = {4.0407f, 6.2914f, 0.4290f};
    float __inertias[DOF] = {0.5f, 5.0f, 5.0f};
    float __fc_coeffs[DOF] = {1.0f, 1.0f, 1.0f};
    float __fv_coeffs[DOF] = {50.0f, 50.0f, 50.0f};
}
#endif  // ROBOT_DEFINITION_H
