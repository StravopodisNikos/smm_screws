#include "smm_screws/core/ScrewsKinematicsNdof.h"

// ============ Ndof version of setExponentials ==================
//
// [14-7-24] Calculates the active joints exponentials
//           Uses the reference anatomy active twists,
//           MUST be used in POE expressions WITH pseudo tfs
//
// Direct Ndof translation of your 3-DOF code:
//   for (size_t i = 0; i < robot_params::DOF; i++)
//       _active_expos[i] = twistExp(_ptr2abstract->active_twists[i], q[i]);
//
void ScrewsKinematicsNdof::setExponentials(float* q)
{
    if (!q) {
        std::cerr << "[ScrewsKinematicsNdof::setExponentials] WARNING: q is null\n";
        return;
    }

    for (int i = 0; i < _dof; ++i) {
        _active_expos[i] = twistExp(_ptr2abstract_ndof->active_twists[i], q[i]);
    }
}

// ============ Ndof version of setExponentialsAnat ===============
//
// [14-7-24] Calculates the active joints exponentials
//           Uses the test anatomy active twists,
//           NO pseudo tfs should be included in the
//           POE expressions.
//           Active twists for test anatomy must be calculated!
//
// Direct Ndof translation of your 3-DOF code:
//   for (size_t i = 0; i < robot_params::DOF; i++)
//       _active_expos_anat[i] = twistExp(_ptr2abstract->active_twists_anat[i], q[i]);
//
void ScrewsKinematicsNdof::setExponentialsAnat(float* q)
{
    if (!q) {
        std::cerr << "[ScrewsKinematicsNdof::setExponentialsAnat] WARNING: q is null\n";
        return;
    }

    for (int i = 0; i < _dof; ++i) {
        _active_expos_anat[i] =
            twistExp(_ptr2abstract_ndof->active_twists_anat[i], q[i]);
    }
}
