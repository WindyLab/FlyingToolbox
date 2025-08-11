//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 23-May-2024 20:24:18
//

#ifndef PREDICT_H
#define PREDICT_H

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

// Function Declarations
namespace coder {
namespace internal {
namespace ctarget {
float DeepLearningNetwork_predict(DeepLearningNetwork &obj,
                                  const double varargin_1[6]);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for predict.h
//
// [EOF]
//
