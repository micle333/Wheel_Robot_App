/*
 * File: threewheel_types.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 14-Apr-2018 21:25:09
 */

#ifndef THREEWHEEL_TYPES_H
#define THREEWHEEL_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_OutStruct
#define typedef_OutStruct

typedef struct {
  float Rn[2];
  float Lat;
  float Lon;
  float Vb;
  float Cbn[9];
  float Heading;
  float Wb;
} OutStruct;

#endif                                 /*typedef_OutStruct*/

#ifndef typedef_ParametersEKFStruct
#define typedef_ParametersEKFStruct

typedef struct {
  float nlr;
  float nll;
  float nw;
  float nlre;
  float nlle;
  float nwe;
} ParametersEKFStruct;

#endif                                 /*typedef_ParametersEKFStruct*/

#ifndef typedef_ParametersGPSStruct
#define typedef_ParametersGPSStruct

typedef struct {
  unsigned char update;
  float lb[3];
  float pos_noise;
  float vel_noise;
  float lat0;
  float lon0;
  float a;
  float e;
} ParametersGPSStruct;

#endif                                 /*typedef_ParametersGPSStruct*/

#ifndef typedef_ParametersGYRStruct
#define typedef_ParametersGYRStruct

typedef struct {
  unsigned char update;
  float noise;
} ParametersGYRStruct;

#endif                                 /*typedef_ParametersGYRStruct*/

#ifndef typedef_ParametersINIStruct
#define typedef_ParametersINIStruct

typedef struct {
  float Rn[2];
  float heading;
  float dw;
  float dlr;
  float dll;
} ParametersINIStruct;

#endif                                 /*typedef_ParametersINIStruct*/

#ifndef typedef_ParametersSLAMStruct
#define typedef_ParametersSLAMStruct

typedef struct {
  unsigned char update;
  float pos_noise;
  float heading_noise;

} ParametersSLAMStruct;

#endif                                 /*typedef_ParametersSLAMtruct*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  float lr;
  float ll;
  float d;
  float thr;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_ParametersStruct
#define typedef_ParametersStruct

typedef struct {
  float dt;
  struct0_T odo;
  ParametersGPSStruct gps;
  ParametersGYRStruct gyr;
  ParametersINIStruct ini;
  ParametersEKFStruct ekf;
  ParametersSLAMStruct slam;
  unsigned char reset;
} ParametersStruct;

#endif                                 /*typedef_ParametersStruct*/

#ifndef typedef_SensorsGPSStruct
#define typedef_SensorsGPSStruct

typedef struct {
  float pos[2];
  float vel[2];
} SensorsGPSStruct;

#endif                                 /*typedef_SensorsGPSStruct*/

#ifndef typedef_SensorsGYRStruct
#define typedef_SensorsGYRStruct

typedef struct {
  float dthe;
} SensorsGYRStruct;

#endif                                 /*typedef_SensorsGYRStruct*/

#ifndef typedef_SensorsODOStruct
#define typedef_SensorsODOStruct

typedef struct {
  float fir;
  float fil;
} SensorsODOStruct;

#endif                                 /*typedef_SensorsODOStruct*/

#ifndef typedef_SensorsSLAMStruct
#define typedef_SensorsSLAMStruct

typedef struct {
  float dthe;
  float pos[2];
} SensorsSLAMStruct;

#endif                                 /*typedef_SensorsSLAMStruct*/

#ifndef typedef_SensorsStruct
#define typedef_SensorsStruct

typedef struct {
  SensorsODOStruct odo;
  SensorsGYRStruct gyr;
  SensorsGPSStruct gps;
  SensorsSLAMStruct slam;
} SensorsStruct;

#endif                                 /*typedef_SensorsStruct*/

#ifndef typedef_StateStruct
#define typedef_StateStruct

typedef struct {
  float P[36];
  float Rn[2];
  float Cbn[9];
  float dw;
  float dlr;
  float dll;
  unsigned char state;
} StateStruct;

#endif                                 /*typedef_StateStruct*/
#endif

/*
 * File trailer for threewheel_types.h
 *
 * [EOF]
 */
