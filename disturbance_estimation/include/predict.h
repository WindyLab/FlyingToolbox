//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 2025-08-08 19:18:20
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
                                  const double varargin_1[8]);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for predict.h
//
// [EOF]
//
