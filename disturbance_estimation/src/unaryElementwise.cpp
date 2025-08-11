//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: unaryElementwise.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 2025-08-08 19:18:20
//

// Include Files
#include "unaryElementwise.h"
#include "omp.h"
#include <cmath>
#include <xmmintrin.h>

// Function Declarations
static void elementwise_relu(const float &inputTensor, float &outputTensor);

// Function Definitions
//
// Arguments    : const float &inputTensor
//                float &outputTensor
// Return Type  : void
//
static void elementwise_relu(const float &inputTensor, float &outputTensor)
{
  __m128 reg_0;
  __m128 reluZero;
  int baseIdx;
  reluZero = _mm_set1_ps(0.0F);
#pragma omp parallel for num_threads(omp_get_max_threads()) private(baseIdx,   \
                                                                        reg_0)

  for (int simdBlockIdx = 0; simdBlockIdx < 12; simdBlockIdx++) {
    baseIdx = simdBlockIdx << 2;
    reg_0 = _mm_loadu_ps(&(&inputTensor)[baseIdx]);
    reg_0 = _mm_max_ps(reg_0, reluZero);
    _mm_storeu_ps(&(&outputTensor)[baseIdx], reg_0);
  }
  (&outputTensor)[48] = std::fmax((&inputTensor)[48], 0.0F);
  (&outputTensor)[49] = std::fmax((&inputTensor)[49], 0.0F);
}

//
// Arguments    : const float X[50]
//                float Z[50]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace layer {
namespace optimized {
void unaryElementwise(const float X[50], float Z[50])
{
  elementwise_relu(X[0], Z[0]);
}

} // namespace optimized
} // namespace layer
} // namespace internal
} // namespace coder

//
// File trailer for unaryElementwise.cpp
//
// [EOF]
//
