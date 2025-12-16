/*
 * File: NMPC_Path_Tracking_ca.h
 *
 * Abstract: Tests assumptions in the generated code.
 */

#ifndef NMPC_PATH_TRACKING_CA_H
#define NMPC_PATH_TRACKING_CA_H

/* preprocessor validation checks */
#include "NMPC_Path_Tracking_ca_preproc.h"
#include "coder_assumptions_hwimpl.h"

/* variables holding test results */
extern CA_HWImpl_TestResults CA_NMPC_Path_Tracking_HWRes;
extern CA_PWS_TestResults CA_NMPC_Path_Tracking_PWSRes;

/* variables holding "expected" and "actual" hardware implementation */
extern const CA_HWImpl CA_NMPC_Path_Tracking_ExpHW;
extern CA_HWImpl CA_NMPC_Path_Tracking_ActHW;

/* entry point function to run tests */
void NMPC_Path_Tracking_caRunTests(void);

#endif                                 /* NMPC_PATH_TRACKING_CA_H */
