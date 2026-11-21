#pragma once

#include "vex.h"


constexpr double leftDistFwdMm   = 44.45;
constexpr double leftDistLeftMm  = 146.5; 
constexpr double frontDistFwdMm  = 196.85;  
constexpr double frontDistLeftMm = 107.95; 

constexpr double fieldSizeMm = 3657.6; 
constexpr double fieldHalfMm = 1828.8;    
constexpr double wallXMinMm  = -fieldHalfMm;
constexpr double wallXMaxMm  =  fieldHalfMm;
constexpr double wallYMinMm  = -fieldHalfMm;
constexpr double wallYMaxMm  =  fieldHalfMm;

constexpr double wallResetMinRangeMm       = 30.0;
constexpr double wallResetMaxRangeMm       = 900.0;
constexpr double wallResetMaxIncidenceRad  = 0.35;  
constexpr double wallResetMaxCorrectionMm  = 600.0;  

constexpr int    maxWallSamples            = 16;
constexpr int    wallSampleMs              = 15;

bool resetPositionFromWalls(bool useLeft = true, bool useFront = true, int samples = 8);

void wallCalibrationMode();
