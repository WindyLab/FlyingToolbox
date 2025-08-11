//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: elementwiseOperationInPlace.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 24-May-2024 14:17:44
//

// Include Files
#include "elementwiseOperationInPlace.h"
#include "omp.h"
#include <cmath>

// Function Definitions
//
// Arguments    : float X[64]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace layer {
void lambdaForColumnMajorWithOMP(float X[64])
{
#pragma omp parallel for num_threads(omp_get_max_threads())

  for (int iElem = 0; iElem < 64; iElem++) {
    X[iElem] = std::fmax(0.0F, X[iElem]);
  }
}

} // namespace layer
} // namespace internal
} // namespace coder

//
// File trailer for elementwiseOperationInPlace.cpp
//
// [EOF]
//
