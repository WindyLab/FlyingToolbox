//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 2025-08-08 19:18:20
//

// Include Files
#include "predict.h"
#include "nn_predict_internal_types.h"
#include "predictForRNN.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : DeepLearningNetwork &obj
//                const double varargin_1[8]
// Return Type  : float
//
namespace coder {
namespace internal {
namespace ctarget {
float DeepLearningNetwork_predict(DeepLearningNetwork &obj,
                                  const double varargin_1[8])
{
  cell_wrap_3 r;
  std::copy(&varargin_1[0], &varargin_1[8], &r.f1[0]);
  return DeepLearningNetwork_predictForRNN(obj, r);
}

} // namespace ctarget
} // namespace internal
} // namespace coder

//
// File trailer for predict.cpp
//
// [EOF]
//
