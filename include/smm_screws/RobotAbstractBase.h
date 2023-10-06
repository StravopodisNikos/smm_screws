#ifndef ROBOT_ABSTRACT_BASE_H
#define ROBOT_ABSTRACT_BASE_H

#include <Eigen/Dense>
#include <Eigen/Core>

#define DOF 3
#define METALINKS 2

// This is the base class that contains the funtamental geometry properties 
// of any serial metamorphic structure. Currently 3dof structures are suppo-
// rted with 2 metamorphic links.
class RobotAbstractBase {
public:
    // DATA COMMON FOR ALL:
    // 1. the active joints twists
    // 2. the home configuration homogeneous tfs(joints+tool)
    Eigen::Matrix<float, 6, 1> active_twists[DOF]; 
    Eigen::Isometry3f* gsai_ptr[DOF+1]; // matrix of pointers to the arrays of the joint tfs + gst @ zero configuration
    // Use Eigen::Isometry3f gsa10; && matrices[0] = &gsa10; ... in the cpp file to initialize.

    // FUNCTIONS
    virtual ~RobotAbstractBase() {}
    virtual  uint8_t get_STRUCTURE_ID()  = 0;
    virtual  uint8_t get_PSEUDOS_METALINK1()  = 0;
    virtual  uint8_t get_PSEUDOS_METALINK2()  = 0;
    virtual  float get_PSEUDO_ANGLES(int index)  = 0;
    virtual  Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index)  = 0;
};

// This is a "dummy" class, but important if you want to use the ros_pkg for 
// fixed-structure serial manipulators. Every virtual function of the abstract
// class must be implemented (for compile issues), but "0" values are returned.
class FixedStructure : public RobotAbstractBase {
public:
    static constexpr int SMM_PSEUDOS = 0;
    int META1_PSEUDOS = 0;
    int META2_PSEUDOS = 0;
    float pseudo_angles = 0;
    Eigen::Matrix<float, 6, 1> passive_twists;

    uint8_t get_STRUCTURE_ID() { return SMM_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK1() { return META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2() { return META2_PSEUDOS; }
    float get_PSEUDO_ANGLES(int index) {return pseudo_angles;}
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) { return passive_twists;}
};

// This the Class that handles the structures with 2 pseudojoints. In this case
// it is obvious that each metamorphic link has 1 pseudojoint. Despite that, the
// variables META1_PSEUDOS,META2_PSEUDOS are not defined "static constexpr" for
// following the same logic with the next structures with more pseudojoints.
class Structure2Pseudos : public RobotAbstractBase {
public:
    static constexpr int SMM_PSEUDOS = 2; // static, no change after compile
    uint8_t META1_PSEUDOS; // must change in node (in the class object construction)
    uint8_t META2_PSEUDOS; // " ... "
    float pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t get_STRUCTURE_ID()  { return SMM_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK1()  { return META1_PSEUDOS; }
    uint8_t get_PSEUDOS_METALINK2()  { return META2_PSEUDOS; }
    float get_PSEUDO_ANGLES(int index)  { return pseudo_angles[index]; }
    Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) { return passive_twists[index]; }
};
// to be expanded soon...
/*
class Structure3Pseudos : public RobotAbstractBase {
public:
    static constexpr int SMM_PSEUDOS = 3;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t get_STRUCTURE_ID() const override { return SMM_PSEUDOS; }
    const Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) const override {
        // Ensure index is within bounds (0 <= index < SMM_PSEUDOS)
        if (index >= 0 && index < SMM_PSEUDOS) {
            return passive_twists[index];
        } else {
            // Return some default value or handle the error as per your requirement
            // Here, we return the first passive_twists by default
            return passive_twists[0];
        }
    }
    const float getPSEUDO_ANGLE(int index) const override {
        return pseudo_angles[index];
    }
};

class Structure4Pseudos : public RobotAbstractBase {
public:
    static constexpr int SMM_PSEUDOS = 4;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t get_STRUCTURE_ID() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> get_PASSIVE_TWISTS(int index) const override {
        // Ensure index is within bounds (0 <= index < SMM_PSEUDOS)
        if (index >= 0 && index < SMM_PSEUDOS) {
            return passive_twists[index];
        } else {
            // Return some default value or handle the error as per your requirement
            // Here, we return the first passive_twists by default
            return passive_twists[0];
        }
    }
    const float getPSEUDO_ANGLE(int index) const override {
        return pseudo_angles[index];
    }
};
*/
#endif 