//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: nn_predict.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 24-May-2024 14:17:44
//

// Include Files
#include "nn_predict.h"
#include "nn_predict_data.h"
#include "nn_predict_initialize.h"
#include "nn_predict_internal_types.h"
#include "predict.h"

// Variable Definitions
static coder::internal::ctarget::DeepLearningNetwork disnet;

static boolean_T disnet_not_empty;

// Function Definitions
//
// A persistent object disnet is used to load the series network object.
//  At the first call to this function, the persistent object is constructed and
//  setup. When the function is called subsequent times, the same object is
//  reused to call predict on inputs, thus avoiding reconstructing and reloading
//  the network object.
//
// Arguments    : const double in[8]
// Return Type  : float
//
float nn_predict(const double in[8])
{
  if (!isInitialized_nn_predict) {
    nn_predict_initialize();
  }
  if (!disnet_not_empty) {
    disnet.IsNetworkInitialized = false;
    disnet.matlabCodegenIsDeleted = false;
    disnet_not_empty = true;
  }
  //  pass in input
  return coder::internal::ctarget::DeepLearningNetwork_predict(disnet, in);
}

//
// A persistent object disnet is used to load the series network object.
//  At the first call to this function, the persistent object is constructed and
//  setup. When the function is called subsequent times, the same object is
//  reused to call predict on inputs, thus avoiding reconstructing and reloading
//  the network object.
//
// Arguments    : void
// Return Type  : void
//
void nn_predict_delete()
{
  if (!disnet.matlabCodegenIsDeleted) {
    disnet.matlabCodegenIsDeleted = true;
  }
}

//
// A persistent object disnet is used to load the series network object.
//  At the first call to this function, the persistent object is constructed and
//  setup. When the function is called subsequent times, the same object is
//  reused to call predict on inputs, thus avoiding reconstructing and reloading
//  the network object.
//
// Arguments    : void
// Return Type  : void
//
void nn_predict_init()
{
  disnet_not_empty = false;
  disnet.matlabCodegenIsDeleted = true;
}

//
// File trailer for nn_predict.cpp
//
// [EOF]
//
