#ifndef ROBOT_ABSTRACT_BASE_H
#define ROBOT_ABSTRACT_BASE_H

#include <Eigen/Dense>
#include <Eigen/Core>

class RobotAbstractBase {
public:
    // DATA COMMON FOR ALL:
    // 1. the active joints twists
    // 2. the home configuration homogeneous tfs(joints+tool)
    Eigen::Matrix<float, 6, 1> active_twists[3]; 
    Eigen::Isometry3f* gsai_ptr[4]; // matrix of pointers to the arrays of the joint tfs + gst @ zero configuration
    // Use Eigen::Isometry3f gsa10; && matrices[0] = &gsa10; ... in the cpp file to initialize.

    // FUNCTIONS
    virtual ~RobotAbstractBase() {}
    virtual uint8_t getPSEUDOS() const = 0;
    virtual const float getPSEUDO_ANGLE(int index) const = 0;
    virtual const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const = 0;
};

class Structure2Pseudos : public RobotAbstractBase {
public:
    static constexpr int SMM_PSEUDOS = 2;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const override {
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

class Structure3Pseudos : public RobotAbstractBase {
public:
    static constexpr int SMM_PSEUDOS = 3;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const override {
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

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const override {
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

#endif 