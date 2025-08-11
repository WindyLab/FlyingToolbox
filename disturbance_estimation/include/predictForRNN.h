//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictForRNN.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 2025-08-08 19:18:20
//

#ifndef PREDICTFORRNN_H
#define PREDICTFORRNN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace internal {
namespace ctarget {
struct DeepLearningNetwork;

}
} // namespace internal
} // namespace coder
struct cell_wrap_3;

// Function Declarations
namespace coder {
namespace internal {
namespace ctarget {
float DeepLearningNetwork_predictForRNN(DeepLearningNetwork &obj,
                                        const cell_wrap_3 inputs);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for predictForRNN.h
//
// [EOF]
//
