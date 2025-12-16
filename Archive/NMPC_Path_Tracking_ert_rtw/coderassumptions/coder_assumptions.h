/*
 * File: coder_assumptions.h
 *
 * Abstract: Coder assumptions header file
 */

#ifndef CODER_ASSUMPTIONS_H
#define CODER_ASSUMPTIONS_H

/* include model specific checks */
#include "NMPC_Path_Tracking_ca.h"

/* global results variable mapping for static code */
#define CA_Expected_HWImpl             CA_NMPC_Path_Tracking_ExpHW
#define CA_Actual_HWImpl               CA_NMPC_Path_Tracking_ActHW
#define CA_HWImpl_Results              CA_NMPC_Path_Tracking_HWRes
#define CA_PortableWordSizes_Results   CA_NMPC_Path_Tracking_PWSRes

/* entry point function mapping for static code */
#define CA_Run_Tests                   NMPC_Path_Tracking_caRunTests
#endif                                 /* CODER_ASSUMPTIONS_H */
