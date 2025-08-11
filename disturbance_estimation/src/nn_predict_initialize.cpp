//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: nn_predict_initialize.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 24-May-2024 14:17:44
//

// Include Files
#include "nn_predict_initialize.h"
#include "nn_predict.h"
#include "nn_predict_data.h"
#include "omp.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void nn_predict_initialize()
{
  omp_init_nest_lock(&nn_predict_nestLockGlobal);
  nn_predict_init();
  isInitialized_nn_predict = true;
}

//
// File trailer for nn_predict_initialize.cpp
//
// [EOF]
//
