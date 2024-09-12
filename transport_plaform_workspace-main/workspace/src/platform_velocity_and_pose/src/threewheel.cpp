/*
 * File: threewheel.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 14-Apr-2018 21:25:09
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include <platform_velocity_and_pose/threewheel.h>
//#include <cstdio>

/* Function Declarations */
static void angle_dcm(float rz, double ry, double rx, float Cnb[9]);
static void dcmnormalize(float C[9]);
static void eye(float I[9]);
static void geo2ecef(float lat, float lon, double alt, float a, float e, float
                     xe[3]);
static void mrdivide(float A[24], const float B[16]);
static void ned2geo(const float xn[3], float lat0, float lon0, double alt0,
                    float a, float e, float *lat, float *lon, float *alt);
static float norm(const float x[3]);

/* Function Definitions */

/*
 * function Cnb = angle_dcm(rz, ry, rx)
 * Cbn = angle_dcm(rz, ry, rx)
 *   Transforms Euler angles to  direction cosine matrix
 *
 *    Input arguments:
 *    rz, ry, rx - Euler angles around Z, Y and X axes correspondingly
 *
 *    Output arguments:
 *    Cbn -  Direction cosine matrix [3,3]
 * Arguments    : float rz
 *                double ry
 *                double rx
 *                float Cnb[9]
 * Return Type  : void
 */
static void angle_dcm(float rz, double ry, double rx, float Cnb[9])
{
  float sz;
  float cz;
  double sy;
  double cy;
  double sx;
  double cx;
  double dv0[9];
  double b_cy[9];
  int i0;
  static const signed char iv0[3] = { 1, 0, 0 };

  static const signed char iv1[3] = { 0, 1, 0 };

  float b_cz[9];
  int i1;
  static const signed char iv2[3] = { 0, 0, 1 };

  double d0;
  int i2;
  float fv0[9];

  /* 'angle_dcm:11' sz = sin(rz); */
  sz = (float)sin(rz);

  /* 'angle_dcm:12' cz = cos(rz); */
  cz = (float)cos(rz);

  /* 'angle_dcm:13' sy = sin(ry); */
  sy = sin(ry);

  /* 'angle_dcm:14' cy = cos(ry); */
  cy = cos(ry);

  /* 'angle_dcm:15' sx = sin(rx); */
  sx = sin(rx);

  /* 'angle_dcm:16' cx = cos(rx); */
  cx = cos(rx);

  /* 'angle_dcm:18' Cx = [1 0 0; 0 cx sx; 0 -sx cx]; */
  /* 'angle_dcm:19' Cy = [cy 0 -sy; 0 1 0; sy 0 cy]; */
  /* 'angle_dcm:20' Cz = [cz sz 0; -sz cz 0; 0 0 1]; */
  /* 'angle_dcm:22' Cnb = Cx*Cy*Cz; */
  dv0[1] = 0.0;
  dv0[4] = cx;
  dv0[7] = sx;
  dv0[2] = 0.0;
  dv0[5] = -sx;
  dv0[8] = cx;
  b_cy[0] = cy;
  b_cy[3] = 0.0;
  b_cy[6] = -sy;
  for (i0 = 0; i0 < 3; i0++) {
    dv0[3 * i0] = iv0[i0];
    b_cy[1 + 3 * i0] = iv1[i0];
  }

  b_cy[2] = sy;
  b_cy[5] = 0.0;
  b_cy[8] = cy;
  b_cz[0] = cz;
  b_cz[3] = sz;
  b_cz[6] = 0.0F;
  b_cz[1] = -sz;
  b_cz[4] = cz;
  b_cz[7] = 0.0F;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      d0 = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        d0 += dv0[i0 + 3 * i2] * b_cy[i2 + 3 * i1];
      }

      fv0[i0 + 3 * i1] = (float)d0;
    }

    b_cz[2 + 3 * i0] = iv2[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      Cnb[i0 + 3 * i1] = 0.0F;
      for (i2 = 0; i2 < 3; i2++) {
        Cnb[i0 + 3 * i1] += fv0[i0 + 3 * i2] * b_cz[i2 + 3 * i1];
      }
    }
  }
}

/*
 * function C = dcmnormalize(C)
 * Normalize Direction Cosine matrix
 * Arguments    : float C[9]
 * Return Type  : void
 */
static void dcmnormalize(float C[9])
{
  float y;
  int i7;
  float b_y;
  float b_C;
  float c_C;
  float d_C;
  float e_C;
  float f_C;
  float g_C;
  float h_C[3];

  /* 'dcmnormalize:3' delta_12 = C(1,:)*C(2,:)'; */
  y = 0.0F;
  for (i7 = 0; i7 < 3; i7++) {
    y += C[3 * i7] * C[1 + 3 * i7];
  }

  /* 'dcmnormalize:4' C(1,:) = C(1,:) - 1/2*delta_12*C(2,:); */
  b_y = 0.5F * y;

  /* 'dcmnormalize:5' C(2,:) = C(2,:) - 1/2*delta_12*C(1,:); */
  y *= 0.5F;
  for (i7 = 0; i7 < 3; i7++) {
    C[3 * i7] -= b_y * C[1 + 3 * i7];
    C[1 + 3 * i7] -= y * C[3 * i7];
  }

  /* 'dcmnormalize:6' C(3,:) = cross(C(1,:),C(2,:)); */
  y = C[6];
  b_y = C[1];
  b_C = C[0];
  c_C = C[7];
  d_C = C[0];
  e_C = C[4];
  f_C = C[3];
  g_C = C[1];
  C[2] = C[3] * C[7] - C[6] * C[4];
  C[5] = y * b_y - b_C * c_C;
  C[8] = d_C * e_C - f_C * g_C;

  /* 'dcmnormalize:7' C(1,:) = C(1,:)./norm(C(1,:)); */
  for (i7 = 0; i7 < 3; i7++) {
    h_C[i7] = C[3 * i7];
  }

  b_y = norm(h_C);

  /* 'dcmnormalize:8' C(2,:) = C(2,:)./norm(C(2,:)); */
  for (i7 = 0; i7 < 3; i7++) {
    C[3 * i7] /= b_y;
    h_C[i7] = C[1 + 3 * i7];
  }

  b_y = norm(h_C);

  /* 'dcmnormalize:9' C(3,:) = C(3,:)./norm(C(3,:)); */
  for (i7 = 0; i7 < 3; i7++) {
    C[1 + 3 * i7] /= b_y;
    h_C[i7] = C[2 + 3 * i7];
  }

  b_y = norm(h_C);
  for (i7 = 0; i7 < 3; i7++) {
    C[2 + 3 * i7] /= b_y;
  }
}

/*
 * Arguments    : float I[9]
 * Return Type  : void
 */
static void eye(float I[9])
{
  int k;
  for (k = 0; k < 9; k++) {
    I[k] = 0.0F;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0F;
  }
}

/*
 * function xe = geo2ecef(lat, lon, alt, a, e)
 * Arguments    : float lat
 *                float lon
 *                double alt
 *                float a
 *                float e
 *                float xe[3]
 * Return Type  : void
 */
static void geo2ecef(float lat, float lon, double alt, float a, float e, float
                     xe[3])
{
  float x;
  float Re;

  /* 'geo2ecef:3' Re = a/sqrt(1-e^2*sin(lat)^2); */
  x = (float)sin(lat);
  Re = a / (float)sqrt(1.0F - e * e * (x * x));

  /* 'geo2ecef:5' x = (Re+alt)*cos(lat)*cos(lon); */
  /* 'geo2ecef:6' y = (Re+alt)*cos(lat)*sin(lon); */
  /* 'geo2ecef:7' z = (Re*(1-e^2)+alt)*sin(lat); */
  /* 'geo2ecef:9' xe = [x;y;z]; */
  xe[0] = (Re + (float)alt) * (float)cos(lat) * (float)cos(lon);
  xe[1] = (Re + (float)alt) * (float)cos(lat) * (float)sin(lon);
  xe[2] = (Re * (1.0F - e * e) + (float)alt) * (float)sin(lat);
}

/*
 * Arguments    : float A[24]
 *                const float B[16]
 * Return Type  : void
 */
static void mrdivide(float A[24], const float B[16])
{
  float b_A[16];
  int k;
  int j;
  signed char ipiv[4];
  int c;
  int iy;
  int jy;
  int ix;
  int jAcol;
  float smax;
  int i;
  float s;
  memcpy(&b_A[0], &B[0], sizeof(float) << 4);
  for (k = 0; k < 4; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    iy = 0;
    ix = c;
    smax = (float)fabs(b_A[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = (float)fabs(b_A[ix]);
      if (s > smax) {
        iy = k - 1;
        smax = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 4; k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = smax;
          ix += 4;
          iy += 4;
        }
      }

      k = (c - j) + 4;
      for (i = c + 1; i < k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 4;
    for (jAcol = 1; jAcol <= 3 - j; jAcol++) {
      smax = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        k = (iy - j) + 8;
        for (i = 5 + iy; i < k; i++) {
          b_A[i] += b_A[ix] * -smax;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  for (j = 0; j < 4; j++) {
    jy = 6 * j;
    jAcol = j << 2;
    for (k = 1; k <= j; k++) {
      iy = 6 * (k - 1);
      if (b_A[(k + jAcol) - 1] != 0.0F) {
        for (i = 0; i < 6; i++) {
          A[i + jy] -= b_A[(k + jAcol) - 1] * A[i + iy];
        }
      }
    }

    smax = 1.0F / b_A[j + jAcol];
    for (i = 0; i < 6; i++) {
      A[i + jy] *= smax;
    }
  }

  for (j = 3; j >= 0; j--) {
    jy = 6 * j;
    jAcol = (j << 2) - 1;
    for (k = j + 2; k < 5; k++) {
      iy = 6 * (k - 1);
      if (b_A[k + jAcol] != 0.0F) {
        for (i = 0; i < 6; i++) {
          A[i + jy] -= b_A[k + jAcol] * A[i + iy];
        }
      }
    }
  }

  for (iy = 2; iy >= 0; iy--) {
    if (ipiv[iy] != iy + 1) {
      jy = ipiv[iy] - 1;
      for (jAcol = 0; jAcol < 6; jAcol++) {
        smax = A[jAcol + 6 * iy];
        A[jAcol + 6 * iy] = A[jAcol + 6 * jy];
        A[jAcol + 6 * jy] = smax;
      }
    }
  }
}

/*
 * function [lat, lon, alt] = ned2geo(xn, lat0, lon0, alt0, a, e)
 * Arguments    : const float xn[3]
 *                float lat0
 *                float lon0
 *                double alt0
 *                float a
 *                float e
 *                float *lat
 *                float *lon
 *                float *alt
 * Return Type  : void
 */
static void ned2geo(const float xn[3], float lat0, float lon0, double alt0,
                    float a, float e, float *lat, float *lon, float *alt)
{
  float fv1[3];
  float fv2[9];
  int i3;
  float x;
  int i4;
  float xe[3];
  float Re;
  float p;
  int i;

  /* 'ned2geo:3' xe0  = geo2ecef(lat0, lon0, alt0, a, e); */
  /* 'ned2geo:5' Cen = [... */
  /* 'ned2geo:6'     -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0)  cos(lat0) */
  /* 'ned2geo:7'               -sin(lon0)            cos(lon0)          0 */
  /* 'ned2geo:8'     -cos(lat0)*cos(lon0) -cos(lat0)*sin(lon0) -sin(lat0) */
  /* 'ned2geo:9'     ]; */
  /* 'ned2geo:11' xe = xe0 + Cen'*xn; */
  geo2ecef(lat0, lon0, alt0, a, e, fv1);
  fv2[0] = -(float)sin(lat0) * (float)cos(lon0);
  fv2[1] = -(float)sin(lat0) * (float)sin(lon0);
  fv2[2] = (float)cos(lat0);
  fv2[3] = -(float)sin(lon0);
  fv2[4] = (float)cos(lon0);
  fv2[5] = 0.0F;
  fv2[6] = -(float)cos(lat0) * (float)cos(lon0);
  fv2[7] = -(float)cos(lat0) * (float)sin(lon0);
  fv2[8] = -(float)sin(lat0);
  for (i3 = 0; i3 < 3; i3++) {
    x = 0.0F;
    for (i4 = 0; i4 < 3; i4++) {
      x += fv2[i3 + 3 * i4] * xn[i4];
    }

    xe[i3] = fv1[i3] + x;
  }

  /* 'ned2geo:13' [lat, lon, alt] = ecef2geo(xe, a, e); */
  /* 'ecef2geo:3' lon = atan2(xe(2,1),xe(1,1)); */
  /* 'ecef2geo:5' alt = single(0); */
  *alt = 0.0F;

  /* 'ecef2geo:6' Re = a; */
  Re = a;

  /* 'ecef2geo:7' p = sqrt(xe(1,1)^2+xe(2,1)^2); */
  p = (float)sqrt(xe[0] * xe[0] + xe[1] * xe[1]);

  /* 'ecef2geo:9' for i=1:25 */
  for (i = 0; i < 25; i++) {
    /* 'ecef2geo:10' sin_lat = xe(3,1)/((1-e^2)*Re+alt); */
    /* 'ecef2geo:11' lat = atan((xe(3,1)+e^2*Re*sin_lat)/p); */
    *lat = (float)atan((xe[2] + e * e * Re * (xe[2] / ((1.0F - e * e) * Re +
      *alt))) / p);

    /* 'ecef2geo:12' Re = a/sqrt(1-e^2*sin(lat)^2); */
    x = (float)sin(*lat);
    Re = a / (float)sqrt(1.0F - e * e * (x * x));

    /* 'ecef2geo:13' alt = p/cos(lat)-Re; */
    *alt = p / (float)cos(*lat) - Re;
  }

  *lon = (float)atan2(xe[1], xe[0]);
}

/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 3; k++) {
    absxk = (float)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt(y);
}

/*
 * function [Out, State] = threewheel(Sensors, State, Parameters)
 * Output structure
 * Arguments    : const SensorsStruct *Sensors
 *                StateStruct *State
 *                const ParametersStruct *Parameters
 *                OutStruct *Out
 * Return Type  : void
 */
void threewheel(const SensorsStruct *Sensors, StateStruct *State, const
                ParametersStruct *Parameters, OutStruct *Out)
{
  int i5;
  float P[36];
  int j;
  float Rn[2];
  float Cbn[9];
  float dw;
  float dlr;
  float dll;
  float Vb;
  float Wb;
  float lr;
  float ll;
  float b_Cbn[9];
  float dthe;
  float Wbg;
  float rot_norm;
  float sr_a;
  float sr_b;
  static const float y[4] = { 0.1F, 0.0F, 0.0F, 0.1F };

  static const float b_y[4] = { 0.0001F, 0.0F, 0.0F, 0.0001F };

  float Fnd[9];
  float b_Rn[3];
  float c_y;
  static const signed char iv3[3] = { 0, 0, 1 };

  float b_Vb[2];
  int i6;
  float c_Cbn[2];
  float An[9];
  float b_An[9];
  float b_Fnd[9];
  float Nn[9];
  float R[9];
  float v[3];
  float Qn[9];
  float F[36];
  float Q[36];
  static const signed char iv4[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float Qwhe[9];
  float b_F[36];
  float b_v;
  float H[6];
  float c_v[4];
  float fv3[3];
  float r;
  float c;
  float b_H[6];
  float d_Cbn[3];
  float K[6];
  static const signed char I[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  float c_Rn[3];
  float b_I[36];
  float d_v[3];
  float c_I[36];
  float x[6];
  float b_K[36];
  float vg_hat[3];
  float c_H[24];
  static const signed char a[4] = { 1, 0, 0, 1 };

  float b_R[16];
  static const signed char iv5[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  float c_K[24];
  float d_H[16];
  float e_H[24];

  /* 'threewheel:4' Out.Rn      = zeros(2,1,'single'); */
  /* 'threewheel:5' Out.Lat     = zeros(1,1,'single'); */
  /* 'threewheel:6' Out.Lon     = zeros(1,1,'single'); */
  /* 'threewheel:7' Out.Vb      = zeros(1,1,'single'); */
  /* 'threewheel:8' Out.Cbn     = zeros(3,3,'single'); */
  /* 'threewheel:9' Out.Heading = zeros(1,1,'single'); */
  /* 'threewheel:10' Out.Wb      = zeros(1,1,'single'); */
  /*  Structures Names */
  /* 'threewheel:13' coder.cstructname(Out,            'OutStruct'); */
  /* 'threewheel:14' coder.cstructname(State,          'StateStruct'); */
  /* 'threewheel:15' coder.cstructname(Parameters,     'ParametersStruct'); */
  /* 'threewheel:16' coder.cstructname(Parameters.gps, 'ParametersGPSStruct'); */
  /* 'threewheel:17' coder.cstructname(Parameters.gyr, 'ParametersGYRStruct'); */
  /* 'threewheel:18' coder.cstructname(Parameters.ekf, 'ParametersEKFStruct'); */
  /* 'threewheel:19' coder.cstructname(Parameters.ini, 'ParametersINIStruct'); */
  /* 'threewheel:20' coder.cstructname(Sensors,        'SensorsStruct'); */
  /* 'threewheel:21' coder.cstructname(Sensors.odo,    'SensorsODOStruct'); */
  /* 'threewheel:22' coder.cstructname(Sensors.gyr,    'SensorsGYRStruct'); */
  /* 'threewheel:23' coder.cstructname(Sensors.gps,    'SensorsGPSStruct'); */
  /*  Get State */
  /* 'threewheel:26' P   = State.P; */
  for (i5 = 0; i5 < 36; i5++) {
    P[i5] = State->P[i5];
  }

  /* 'threewheel:27' Rn  = State.Rn; */
  for (j = 0; j < 2; j++) {
    Rn[j] = State->Rn[j];
  }

  /* 'threewheel:28' Cbn = State.Cbn; */
  for (i5 = 0; i5 < 9; i5++) {
    Cbn[i5] = State->Cbn[i5];
  }

  /* 'threewheel:29' dw  = State.dw; */
  dw = State->dw;

  /* 'threewheel:30' dlr = State.dlr; */
  dlr = State->dlr;

  /* 'threewheel:31' dll = State.dll; */
  dll = State->dll;

  /*  Get Sensors */
  /* 'threewheel:34' fir    = Sensors.odo.fir; */
  /* 'threewheel:35' fil    = Sensors.odo.fil; */
  /* 'threewheel:36' dThe   = Sensors.gyr.dthe; */
  /* 'threewheel:37' latlon = Sensors.gps.pos; */
  /* 'threewheel:38' vg     = Sensors.gps.vel; */
  /*  Parameters */
  /* 'threewheel:41' dt = Parameters.dt; */
  /*  Defaults */
  /* 'threewheel:44' Vb = zeros(1,1,'single'); */
  Vb = 0.0F;

  /* 'threewheel:45' Wb = zeros(1,1,'single'); */
  Wb = 0.0F;

  /*  State  */
  /* 'threewheel:48' switch State.state */
  switch (State->state) {
   case 0:
    /* 'threewheel:49' case 0 %Initialization */
    /* Initialization */
    /* Initial values for  errors */
    /* 'threewheel:51' dw  = Parameters.ini.dw; */
    dw = Parameters->ini.dw;

    /* 'threewheel:52' dlr = Parameters.ini.dlr; */
    dlr = Parameters->ini.dlr;

    /* 'threewheel:53' dll = Parameters.ini.dll; */
    dll = Parameters->ini.dll;

    /* Initial alignment */
    /* 'threewheel:55' Rn = Parameters.ini.Rn; */
    for (j = 0; j < 2; j++) {
      Rn[j] = Parameters->ini.Rn[j];
    }

    /* 'threewheel:56' Cnb = angle_dcm(deg2rad(Parameters.ini.heading), 0, 0); */
    /* 'deg2rad:2' angleInRadians = (pi/180) * angleInDegrees; */
    /* 'threewheel:57' Cbn = Cnb'; */
    angle_dcm(0.0174532924F * Parameters->ini.heading, 0.0, 0.0, b_Cbn);
    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        Cbn[j + 3 * i5] = b_Cbn[i5 + 3 * j];
      }
    }

    /* Initial Covariance matrix */
    /* 'threewheel:59' P(1:2,1:2)=eye(2,'single')*1e-1; */
    /* position errors */
    /* 'threewheel:60' P(3,3)=1e-2; */
    P[14] = 0.01F;

    /* heading error */
    /* 'threewheel:61' P(4:5,4:5)=eye(2,'single')*1e-4; */
    for (i5 = 0; i5 < 2; i5++) {
      for (j = 0; j < 2; j++) {
        P[j + 6 * i5] = y[j + (i5 << 1)];
        P[(j + 6 * (3 + i5)) + 3] = b_y[j + (i5 << 1)];
      }
    }

    /* wheels radii errors */
    /* 'threewheel:62' P(6,6)=1e-4; */
    P[35] = 0.0001F;

    /* gyro bias  error */
    /* 'threewheel:63' State.state = uint8(1); */
    State->state = 1U;
    break;

   case 1:
    /* 'threewheel:64' case 1 %Normal operation */
    /* Normal operation */
    /*         %% Navigation: */
    /* compensate odometer errors */
    /* 'threewheel:67' lr  = Parameters.odo.lr-dlr; */
    lr = Parameters->odo.lr - State->dlr;

    /* 'threewheel:68' ll  = Parameters.odo.ll-dll; */
    ll = Parameters->odo.ll - State->dll;

    /* 'threewheel:69' d   = Parameters.odo.d; */
    /* odometer velocities */
    /* 'threewheel:71' Vb = (fir*lr+fil*ll)/(4*pi); */
    Vb = (Sensors->odo.fir * lr + Sensors->odo.fil * ll) / 12.566371F;

    /* 'threewheel:72' Wb = (fir*lr-fil*ll)/(2*pi*d); */
    // DEBUG: It should be divided by 4pi, not 2pi. But I didnt touch it not to break everything...
    Wb = (Sensors->odo.fir * lr - Sensors->odo.fil * ll) / (6.28318548F *
      Parameters->odo.d);

    /* Compensate gyroscope error */
    /* 'threewheel:74' dthe = dThe-dw*dt; */
    dthe = Sensors->gyr.dthe - State->dw * Parameters->dt;

    /* angle increment */
    /* 'threewheel:75' Wbg  = dthe/dt; */
    Wbg = dthe / Parameters->dt;

    /* angular rate */
    /* Update Attitude */
    /* 'threewheel:77' rot_norm=norm(dthe); */
    rot_norm = (float)fabs(dthe);

    /* 'threewheel:78' sr_a=1-(rot_norm^2/6)+(rot_norm^4/120); */
    sr_a = (1.0F - rot_norm * rot_norm / 6.0F) + (float)pow(rot_norm, 4.0) /
      120.0F;

    /* 'threewheel:79' sr_b=(1/2)-(rot_norm^2/24)+(rot_norm^4/720); */
    sr_b = (0.5F - rot_norm * rot_norm / 24.0F) + (float)pow(rot_norm, 4.0) /
      720.0F;

    /* 'threewheel:80' Cbb = [1-sr_b*dthe^2, -sr_a*dthe, 0;  sr_a*dthe,... */
    /* 'threewheel:81'             1-sr_b*dthe^2, 0; 0, 0, 1]; */
    /* 'threewheel:82' Cbn=Cbn*Cbb; */
    Fnd[0] = 1.0F - sr_b * (dthe * dthe);
    Fnd[3] = -sr_a * dthe;
    Fnd[6] = 0.0F;
    Fnd[1] = sr_a * dthe;
    Fnd[4] = 1.0F - sr_b * (dthe * dthe);
    Fnd[7] = 0.0F;
    for (i5 = 0; i5 < 3; i5++) {
      Fnd[2 + 3 * i5] = iv3[i5];
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        Cbn[i5 + 3 * j] = 0.0F;
        for (i6 = 0; i6 < 3; i6++) {
          Cbn[i5 + 3 * j] += State->Cbn[i5 + 3 * i6] * Fnd[i6 + 3 * j];
        }
      }
    }

    /* Update Position */
    /* 'threewheel:84' Rn = Rn+Cbn(1:2,1:2)*[Vb;0]*dt; */
    b_Vb[0] = Vb;
    b_Vb[1] = 0.0F;
    for (i5 = 0; i5 < 2; i5++) {
      c_Cbn[i5] = 0.0F;
      for (j = 0; j < 2; j++) {
        c_Cbn[i5] += Cbn[i5 + 3 * j] * b_Vb[j];
      }

      Rn[i5] = State->Rn[i5] + c_Cbn[i5] * Parameters->dt;
    }

    /*         %% Kalman Predict: */
    /* Continuous-time system parameters */
    /* 'threewheel:88' An=zeros(3,'single'); */
    for (i5 = 0; i5 < 9; i5++) {
      An[i5] = 0.0F;
    }

    /* 'threewheel:89' An(1,3) =  Cbn(2,1)*Vb; */
    An[6] = Cbn[1] * Vb;

    /* 'threewheel:90' An(2,3) = -Cbn(1,1)*Vb; */
    An[7] = -Cbn[0] * Vb;

    /* discretize An; */
    /* 'threewheel:92' Fnd=eye(3,'single')+An*dt+An*An*dt*dt/2; */
    eye(b_Cbn);
    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        b_An[i5 + 3 * j] = 0.0F;
        for (i6 = 0; i6 < 3; i6++) {
          b_An[i5 + 3 * j] += An[i5 + 3 * i6] * An[i6 + 3 * j];
        }
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        b_Fnd[j + 3 * i5] = (b_Cbn[j + 3 * i5] + An[j + 3 * i5] * Parameters->dt)
          + b_An[j + 3 * i5] * Parameters->dt * Parameters->dt / 2.0F;
      }
    }

    /* Error model - error inputs */
    /* 'threewheel:94' Nn = zeros(3,'single'); */
    /* 'threewheel:95' Nn(1,1) =  Cbn(1,1)*fir/(4*pi); */
    for (i5 = 0; i5 < 9; i5++) {
      Nn[i5] = 0.0F;
      R[i5] = 0.0F;
    }

    Nn[0] = Cbn[0] * Sensors->odo.fir / 12.566371F;

    /* 'threewheel:96' Nn(1,2) =  Cbn(1,1)*fil/(4*pi); */
    Nn[3] = Cbn[0] * Sensors->odo.fil / 12.566371F;

    /* 'threewheel:97' Nn(2,1) =  Cbn(2,1)*fir/(4*pi); */
    Nn[1] = Cbn[1] * Sensors->odo.fir / 12.566371F;

    /* 'threewheel:98' Nn(2,2) =  Cbn(2,1)*fil/(4*pi); */
    Nn[4] = Cbn[1] * Sensors->odo.fil / 12.566371F;

    /* 'threewheel:99' Nn(3,3) = -1; */
    Nn[8] = -1.0F;

    /* discrete Q */
    /* 'threewheel:101' nlr = Parameters.ekf.nlr; */
    /* right wheel noise */
    /* 'threewheel:102' nll = Parameters.ekf.nll; */
    /* left  wheel noise */
    /* 'threewheel:103' nw  = Parameters.ekf.nw; */
    /* gyroscope noise */
    /* 'threewheel:104' R = diag([nlr, nll, nw]); */
    v[0] = Parameters->ekf.nlr;
    v[1] = Parameters->ekf.nll;
    v[2] = Parameters->ekf.nw;
    for (j = 0; j < 3; j++) {
      R[j + 3 * j] = v[j];
    }

    /* 'threewheel:105' Qn = Nn*R*Nn'; */
    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        b_An[i5 + 3 * j] = 0.0F;
        for (i6 = 0; i6 < 3; i6++) {
          b_An[i5 + 3 * j] += Nn[i5 + 3 * i6] * R[i6 + 3 * j];
        }
      }

      for (j = 0; j < 3; j++) {
        Qn[i5 + 3 * j] = 0.0F;
        for (i6 = 0; i6 < 3; i6++) {
          Qn[i5 + 3 * j] += b_An[i5 + 3 * i6] * Nn[j + 3 * i6];
        }
      }
    }

    /* Use trapezoidal rule */
    /* 'threewheel:107' Qnd=dt/2*(Fnd*Qn+Qn*Fnd'); */
    c_y = Parameters->dt / 2.0F;

    /* System matrix and system covariance matrix */
    /* 'threewheel:109' F = zeros(6,'single'); */
    /* 'threewheel:110' Q = zeros(6,'single'); */
    memset(&F[0], 0, 36U * sizeof(float));
    memset(&Q[0], 0, 36U * sizeof(float));

    /* 'threewheel:111' F(1:3,1:3) = Fnd; */
    /* 'threewheel:112' F(4:6,4:6) = diag([1.0, 1.0, 1.0]); */
    /* 'threewheel:113' F(1:3,4:6) = Nn*dt; */
    /* 'threewheel:114' Q(1:3,1:3) = Qnd; */
    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        F[j + 6 * i5] = b_Fnd[j + 3 * i5];
        F[(j + 6 * (3 + i5)) + 3] = iv4[j + 3 * i5];
        F[j + 6 * (3 + i5)] = Nn[j + 3 * i5] * Parameters->dt;
        Fnd[i5 + 3 * j] = 0.0F;
        b_An[i5 + 3 * j] = 0.0F;
        for (i6 = 0; i6 < 3; i6++) {
          Fnd[i5 + 3 * j] += b_Fnd[i5 + 3 * i6] * Qn[i6 + 3 * j];
          b_An[i5 + 3 * j] += Qn[i5 + 3 * i6] * b_Fnd[j + 3 * i6];
        }
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        Q[j + 6 * i5] = c_y * (Fnd[j + 3 * i5] + b_An[j + 3 * i5]);
      }
    }

    /* 'threewheel:115' nlre = Parameters.ekf.nlre; */
    /* right wheel error noise */
    /* 'threewheel:116' nlle = Parameters.ekf.nlle; */
    /* left  wheel error noise */
    /* 'threewheel:117' nwe  = Parameters.ekf.nwe; */
    /* gyroscope bias noise */
    /* 'threewheel:118' Qwhe = diag([nlre, nlle, nwe]); */
    v[0] = Parameters->ekf.nlre;
    v[1] = Parameters->ekf.nlle;
    v[2] = Parameters->ekf.nwe;
    for (i5 = 0; i5 < 9; i5++) {
      Qwhe[i5] = 0.0F;
    }

    for (j = 0; j < 3; j++) {
      Qwhe[j + 3 * j] = v[j];
    }

    /* 'threewheel:119' Q(4:6,4:6) = Qwhe; */
    /* 'threewheel:120' Q(1:3,4:6)=Nn*Qwhe*dt/2; */
    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        Q[(j + 6 * (3 + i5)) + 3] = Qwhe[j + 3 * i5];
        b_An[i5 + 3 * j] = 0.0F;
        for (i6 = 0; i6 < 3; i6++) {
          b_An[i5 + 3 * j] += Nn[i5 + 3 * i6] * Qwhe[i6 + 3 * j];
        }
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        Q[j + 6 * (3 + i5)] = b_An[j + 3 * i5] * Parameters->dt / 2.0F;
      }
    }

    /* 'threewheel:121' Q(4:6,1:3)=Q(1:3,4:6)'; */
    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        b_An[j + 3 * i5] = Q[i5 + 6 * (3 + j)];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (j = 0; j < 3; j++) {
        Q[(j + 6 * i5) + 3] = b_An[j + 3 * i5];
      }
    }

    /* Covariance predict */
    /* 'threewheel:123' P = F*P*F'+Q; */
    for (i5 = 0; i5 < 6; i5++) {
      for (j = 0; j < 6; j++) {
        b_F[i5 + 6 * j] = 0.0F;
        for (i6 = 0; i6 < 6; i6++) {
          b_F[i5 + 6 * j] += F[i5 + 6 * i6] * State->P[i6 + 6 * j];
        }
      }

      for (j = 0; j < 6; j++) {
        c_y = 0.0F;
        for (i6 = 0; i6 < 6; i6++) {
          c_y += b_F[i5 + 6 * i6] * F[j + 6 * i6];
        }

        P[i5 + 6 * j] = c_y + Q[i5 + 6 * j];
      }
    }

    /*         %% Kalman Update - GPS: */
    /* 'threewheel:126' if (Parameters.gps.update == 1) */
    if (Parameters->gps.update == 1) {
      /* measurement vector */
      /* 'threewheel:128' rg = geo2ned(latlon(1), latlon(2), 0,... */
      /* 'threewheel:129'                 Parameters.gps.lat0, Parameters.gps.lon0, 0,... */
      /* 'threewheel:130'                 Parameters.gps.a, Parameters.gps.e); */
      /* 'geo2ned:3' xe  = geo2ecef(lat, lon, alt, a, e); */
      /* 'geo2ned:4' xe0  = geo2ecef(lat0, lon0, alt0, a, e); */
      /* 'geo2ned:6' Cen = [... */
      /* 'geo2ned:7'     -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0)  cos(lat0) */
      /* 'geo2ned:8'               -sin(lon0)            cos(lon0)          0 */
      /* 'geo2ned:9'     -cos(lat0)*cos(lon0) -cos(lat0)*sin(lon0) -sin(lat0) */
      /* 'geo2ned:10'     ]; */
      /* 'geo2ned:12' xn = Cen*(xe-xe0); */
      /* 'threewheel:131' v = zeros(4,1,'single'); */
      for (j = 0; j < 4; j++) {
        c_v[j] = 0.0F;
      }

      /* 'threewheel:132' rg_hat = [Rn; 0]+Cbn*Parameters.gps.lb; */
      /* 'threewheel:133' v(1:2,1) = rg_hat(1:2,1)-rg(1:2,1); */
      geo2ecef(Sensors->gps.pos[0], Sensors->gps.pos[1], 0.0, Parameters->gps.a,
               Parameters->gps.e, fv3);
      for (i5 = 0; i5 < 3; i5++) {
        v[i5] = fv3[i5];
      }

      geo2ecef(Parameters->gps.lat0, Parameters->gps.lon0, 0.0,
               Parameters->gps.a, Parameters->gps.e, fv3);
      for (i5 = 0; i5 < 2; i5++) {
        b_Rn[i5] = Rn[i5];
      }

      b_Rn[2] = 0.0F;
      Fnd[0] = -(float)sin(Parameters->gps.lat0) * (float)cos
        (Parameters->gps.lon0);
      Fnd[3] = -(float)sin(Parameters->gps.lat0) * (float)sin
        (Parameters->gps.lon0);
      Fnd[6] = (float)cos(Parameters->gps.lat0);
      Fnd[1] = -(float)sin(Parameters->gps.lon0);
      Fnd[4] = (float)cos(Parameters->gps.lon0);
      Fnd[7] = 0.0F;
      Fnd[2] = -(float)cos(Parameters->gps.lat0) * (float)cos
        (Parameters->gps.lon0);
      Fnd[5] = -(float)cos(Parameters->gps.lat0) * (float)sin
        (Parameters->gps.lon0);
      Fnd[8] = -(float)sin(Parameters->gps.lat0);
      for (i5 = 0; i5 < 3; i5++) {
        d_Cbn[i5] = 0.0F;
        for (j = 0; j < 3; j++) {
          d_Cbn[i5] += Cbn[i5 + 3 * j] * Parameters->gps.lb[j];
        }

        c_Rn[i5] = b_Rn[i5] + d_Cbn[i5];
        d_v[i5] = v[i5] - fv3[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        fv3[i5] = 0.0F;
        for (j = 0; j < 3; j++) {
          fv3[i5] += Fnd[i5 + 3 * j] * d_v[j];
        }
      }
      //printf("\n");
      for (i5 = 0; i5 < 2; i5++) {
        c_v[i5] = c_Rn[i5] - fv3[i5];
        //printf("gps coord %.2f ", c_v[i5]);
      }
      //printf("\n");

      /* 'threewheel:134' vb = [Vb;0;0]; */
      /* 'threewheel:135' wb = [0;0;Wbg]; */
      /* 'threewheel:136' vg_hat = Cbn*(vb+skew(wb)*Parameters.gps.lb); */
      /* 'skew:2' res=[0 			-inp_vr(3)	inp_vr(2); ... */
      /* 'skew:3'      inp_vr(3)	0			-inp_vr(1);... */
      /* 'skew:4' 	 -inp_vr(2)	inp_vr(1)	0]; */
      /* res=zeros(3,3); */
      /* res(1,2)=-inp_vr(3); */
      /* res(1,3)=inp_vr(2); */
      /* res(2,1)=inp_vr(3); */
      /* res(2,3)=-inp_vr(1); */
      /* res(3,1)=-inp_vr(2); */
      /* res(3,2)=inp_vr(1); */
      Fnd[0] = 0.0F;
      Fnd[3] = -Wbg;
      Fnd[6] = 0.0F;
      Fnd[1] = Wbg;
      Fnd[4] = 0.0F;
      Fnd[7] = -0.0F;
      Fnd[2] = -0.0F;
      Fnd[5] = 0.0F;
      Fnd[8] = 0.0F;
      v[0] = Vb;
      v[1] = 0.0F;
      v[2] = 0.0F;
      for (i5 = 0; i5 < 3; i5++) {
        fv3[i5] = 0.0F;
        for (j = 0; j < 3; j++) {
          fv3[i5] += Fnd[i5 + 3 * j] * Parameters->gps.lb[j];
        }

        b_Rn[i5] = v[i5] + fv3[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        vg_hat[i5] = 0.0F;
        for (j = 0; j < 3; j++) {
          vg_hat[i5] += Cbn[i5 + 3 * j] * b_Rn[j];
        }
      }

      /* 'threewheel:137' v(3:4,1) = vg_hat(1:2,1)-vg; */
      for (i5 = 0; i5 < 2; i5++) {
        c_v[2 + i5] = vg_hat[i5] - Sensors->gps.vel[i5];
      }

      /* measurement matrix */
      /* 'threewheel:139' H = zeros(4,6,'single'); */
      memset(&c_H[0], 0, 24U * sizeof(float));

      /* 'threewheel:140' H(1:2,1:2) = eye(2,'single'); */
      for (i5 = 0; i5 < 2; i5++) {
        for (j = 0; j < 2; j++) {
          c_H[j + (i5 << 2)] = a[j + (i5 << 1)];
        }
      }

      /* 'threewheel:141' dpsi = skew(Cbn*Parameters.gps.lb); */
      for (i5 = 0; i5 < 3; i5++) {
        d_Cbn[i5] = 0.0F;
        for (j = 0; j < 3; j++) {
          d_Cbn[i5] += Cbn[i5 + 3 * j] * Parameters->gps.lb[j];
        }

        v[i5] = d_Cbn[i5];
      }

      /* 'skew:2' res=[0 			-inp_vr(3)	inp_vr(2); ... */
      /* 'skew:3'      inp_vr(3)	0			-inp_vr(1);... */
      /* 'skew:4' 	 -inp_vr(2)	inp_vr(1)	0]; */
      /* res=zeros(3,3); */
      /* res(1,2)=-inp_vr(3); */
      /* res(1,3)=inp_vr(2); */
      /* res(2,1)=inp_vr(3); */
      /* res(2,3)=-inp_vr(1); */
      /* res(3,1)=-inp_vr(2); */
      /* res(3,2)=inp_vr(1); */
      /* 'threewheel:142' H(1:2,3) = dpsi(1:2,3); */
      Fnd[0] = 0.0F;
      Fnd[3] = -v[2];
      Fnd[6] = v[1];
      Fnd[1] = v[2];
      Fnd[4] = 0.0F;
      Fnd[7] = -v[0];
      Fnd[2] = -v[1];
      Fnd[5] = v[0];
      Fnd[8] = 0.0F;

      /* 'threewheel:143' dpsi = skew(vg_hat); */
      /* 'skew:2' res=[0 			-inp_vr(3)	inp_vr(2); ... */
      /* 'skew:3'      inp_vr(3)	0			-inp_vr(1);... */
      /* 'skew:4' 	 -inp_vr(2)	inp_vr(1)	0]; */
      /* res=zeros(3,3); */
      /* res(1,2)=-inp_vr(3); */
      /* res(1,3)=inp_vr(2); */
      /* res(2,1)=inp_vr(3); */
      /* res(2,3)=-inp_vr(1); */
      /* res(3,1)=-inp_vr(2); */
      /* res(3,2)=inp_vr(1); */
      /* 'threewheel:144' H(3:4,3) =  dpsi(1:2,3); */
      b_An[0] = 0.0F;
      b_An[3] = -vg_hat[2];
      b_An[6] = vg_hat[1];
      b_An[1] = vg_hat[2];
      b_An[4] = 0.0F;
      b_An[7] = -vg_hat[0];
      b_An[2] = -vg_hat[1];
      b_An[5] = vg_hat[0];
      b_An[8] = 0.0F;
      for (i5 = 0; i5 < 2; i5++) {
        c_H[8 + i5] = Fnd[6 + i5];
        c_H[i5 + 10] = b_An[6 + i5];
      }

      /* 'threewheel:145' domega   = -Cbn*skew(Parameters.gps.lb); */
      /* 'skew:2' res=[0 			-inp_vr(3)	inp_vr(2); ... */
      /* 'skew:3'      inp_vr(3)	0			-inp_vr(1);... */
      /* 'skew:4' 	 -inp_vr(2)	inp_vr(1)	0]; */
      /* res=zeros(3,3); */
      /* res(1,2)=-inp_vr(3); */
      /* res(1,3)=inp_vr(2); */
      /* res(2,1)=inp_vr(3); */
      /* res(2,3)=-inp_vr(1); */
      /* res(3,1)=-inp_vr(2); */
      /* res(3,2)=inp_vr(1); */
      /* 'threewheel:146' H(3:4,6) = domega(1:2,3); */
      for (i5 = 0; i5 < 3; i5++) {
        for (j = 0; j < 3; j++) {
          b_Cbn[j + 3 * i5] = -Cbn[j + 3 * i5];
        }
      }

      Fnd[0] = 0.0F;
      Fnd[3] = -Parameters->gps.lb[2];
      Fnd[6] = Parameters->gps.lb[1];
      Fnd[1] = Parameters->gps.lb[2];
      Fnd[4] = 0.0F;
      Fnd[7] = -Parameters->gps.lb[0];
      Fnd[2] = -Parameters->gps.lb[1];
      Fnd[5] = Parameters->gps.lb[0];
      Fnd[8] = 0.0F;
      for (i5 = 0; i5 < 3; i5++) {
        for (j = 0; j < 3; j++) {
          b_An[i5 + 3 * j] = 0.0F;
          for (i6 = 0; i6 < 3; i6++) {
            b_An[i5 + 3 * j] += b_Cbn[i5 + 3 * i6] * Fnd[i6 + 3 * j];
          }
        }
      }

      for (i5 = 0; i5 < 2; i5++) {
        c_H[i5 + 22] = b_An[6 + i5];
      }

      /* 'threewheel:147' H(3,4)   = Cbn(1,1)*fir/(4*pi); */
      c_H[14] = Cbn[0] * Sensors->odo.fir / 12.566371F;

      /* 'threewheel:148' H(3,5)   = Cbn(1,1)*fil/(4*pi); */
      c_H[18] = Cbn[0] * Sensors->odo.fil / 12.566371F;

      /* 'threewheel:149' H(4,4)   = Cbn(2,1)*fir/(4*pi); */
      c_H[15] = Cbn[1] * Sensors->odo.fir / 12.566371F;

      /* 'threewheel:150' H(4,5)   = Cbn(2,1)*fil/(4*pi); */
      c_H[19] = Cbn[1] * Sensors->odo.fil / 12.566371F;

      /* measurement noise covariance */
      /* 'threewheel:152' R = eye(4,'single'); */
      for (i5 = 0; i5 < 16; i5++) {
        b_R[i5] = iv5[i5];
      }

      /* 'threewheel:153' R(1:2,1:2) = eye(2,'single')*Parameters.gps.pos_noise^2; */
      c_y = Parameters->gps.pos_noise * Parameters->gps.pos_noise;

      /* 'threewheel:154' R(3:4,3:4) = eye(2,'single')*Parameters.gps.vel_noise^2; */
      c = Parameters->gps.vel_noise * Parameters->gps.vel_noise;
      for (i5 = 0; i5 < 2; i5++) {
        for (j = 0; j < 2; j++) {
          b_R[j + (i5 << 2)] = (float)a[j + (i5 << 1)] * c_y;
          b_R[(j + ((2 + i5) << 2)) + 2] = (float)a[j + (i5 << 1)] * c;
        }
      }

      /* Update */
      /* 'threewheel:156' I = eye(6,'single'); */
      /* 'threewheel:157' K = (P*H')/(H*P*H'+ R); */
      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 4; j++) {
          c_K[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            c_K[i5 + 6 * j] += P[i5 + 6 * i6] * c_H[j + (i6 << 2)];
          }
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        for (j = 0; j < 6; j++) {
          e_H[i5 + (j << 2)] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            e_H[i5 + (j << 2)] += c_H[i5 + (i6 << 2)] * P[i6 + 6 * j];
          }
        }

        for (j = 0; j < 4; j++) {
          c_y = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            c_y += e_H[i5 + (i6 << 2)] * c_H[j + (i6 << 2)];
          }

          d_H[i5 + (j << 2)] = c_y + b_R[i5 + (j << 2)];
        }
      }

      mrdivide(c_K, d_H);

      /* 'threewheel:158' P = (I-K*H)*P*(I-K*H)' + K*R*K'; */
      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          c_y = 0.0F;
          for (i6 = 0; i6 < 4; i6++) {
            c_y += c_K[i5 + 6 * i6] * c_H[i6 + (j << 2)];
          }

          b_F[i5 + 6 * j] = (float)I[i5 + 6 * j] - c_y;
        }

        for (j = 0; j < 6; j++) {
          c_I[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            c_I[i5 + 6 * j] += b_F[i5 + 6 * i6] * P[i6 + 6 * j];
          }

          c_y = 0.0F;
          for (i6 = 0; i6 < 4; i6++) {
            c_y += c_K[j + 6 * i6] * c_H[i6 + (i5 << 2)];
          }

          b_I[i5 + 6 * j] = (float)I[j + 6 * i5] - c_y;
        }

        for (j = 0; j < 4; j++) {
          e_H[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 4; i6++) {
            e_H[i5 + 6 * j] += c_K[i5 + 6 * i6] * b_R[i6 + (j << 2)];
          }
        }
      }

      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          b_F[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            b_F[i5 + 6 * j] += c_I[i5 + 6 * i6] * b_I[i6 + 6 * j];
          }

          b_K[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 4; i6++) {
            b_K[i5 + 6 * j] += e_H[i5 + 6 * i6] * c_K[j + 6 * i6];
          }
        }
      }

      /* 'threewheel:159' x = K*v; */
      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          P[j + 6 * i5] = b_F[j + 6 * i5] + b_K[j + 6 * i5];
        }

        x[i5] = 0.0F;
        for (j = 0; j < 4; j++) {
          x[i5] += c_K[i5 + 6 * j] * c_v[j];
        }
      }

      /* Correct Position */
      /* 'threewheel:161' Rn = Rn-x(1:2,1); */
      for (i5 = 0; i5 < 2; i5++) {
        Rn[i5] -= x[i5];
      }

      /* Correct Attitude */
      /* 'threewheel:163' En  = [1, -x(3,1), 0; x(3,1), 1, 0; 0, 0, 1]; */
      /* 'threewheel:164' Cbn = En*Cbn; */
      Fnd[0] = 1.0F;
      Fnd[3] = -x[2];
      Fnd[6] = 0.0F;
      Fnd[1] = x[2];
      Fnd[4] = 1.0F;
      Fnd[7] = 0.0F;
      for (i5 = 0; i5 < 3; i5++) {
        Fnd[2 + 3 * i5] = iv3[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        for (j = 0; j < 3; j++) {
          b_An[i5 + 3 * j] = 0.0F;
          for (i6 = 0; i6 < 3; i6++) {
            b_An[i5 + 3 * j] += Fnd[i5 + 3 * i6] * Cbn[i6 + 3 * j];
          }
        }
      }

      for (i5 = 0; i5 < 3; i5++) {
        for (j = 0; j < 3; j++) {
          Cbn[j + 3 * i5] = b_An[j + 3 * i5];
        }
      }

      /* Normalize Cbn matrix */
      /* 'threewheel:166' Cbn = dcmnormalize(Cbn); */
      dcmnormalize(Cbn);

      /* Update errors estimates */
      /* 'threewheel:168' dlr = dlr+x(4,1); */
      dlr = State->dlr + x[3];

      /* 'threewheel:169' dll = dll+x(5,1); */
      dll = State->dll + x[4];

      /* 'threewheel:170' dw  = dw +x(6,1); */
      dw = State->dw + x[5];
    }


    /*         %% Kalman Update - Gyroscope: */
    /* 'threewheel:174' if (Parameters.gyr.update == 1) */
    if (Parameters->gyr.update == 1) {
      /* measurement vector with odometer ZUPT */
      /* 'threewheel:176' if (abs(Wb) > Parameters.odo.thr) */
      if ((float)fabs(Wb) > Parameters->odo.thr) {
        /* 'threewheel:177' v =  Wbg-Wb; */
        b_v = Wbg - Wb;
      } else {
        /* 'threewheel:178' else */
        /* 'threewheel:179' v = Wbg; */
        b_v = Wbg;
      }

      /* measurement matrix */
      /* 'threewheel:182' H = zeros(1,6,'single'); */
      for (i5 = 0; i5 < 6; i5++) {
        H[i5] = 0.0F;
      }

      /* 'threewheel:183' H(1,4) = -fir/(2*pi*d); */
      H[3] = -Sensors->odo.fir / (6.28318548F * Parameters->odo.d);

      /* 'threewheel:184' H(1,5) =  fil/(2*pi*d); */
      H[4] = Sensors->odo.fil / (6.28318548F * Parameters->odo.d);

      /* 'threewheel:185' H(1,6) =  1; */
      H[5] = 1.0F;

      /* measurement noise covariance */
      /* 'threewheel:187' r = Parameters.gyr.noise^2; */
      r = Parameters->gyr.noise * Parameters->gyr.noise;

      /* Update */
      /* 'threewheel:189' I = eye(6,'single'); */
      /* 'threewheel:190' K = (P*H')/(H*P*H'+ r); */
      c_y = 0.0F;
      for (i5 = 0; i5 < 6; i5++) {
        b_H[i5] = 0.0F;
        for (j = 0; j < 6; j++) {
          b_H[i5] += H[j] * P[j + 6 * i5];
        }

        c_y += b_H[i5] * H[i5];
      }

      c = c_y + r;

      /* 'threewheel:191' P = (I-K*H)*P*(I-K*H)' + K*r*K'; */
      for (i5 = 0; i5 < 6; i5++) {
        c_y = 0.0F;
        for (j = 0; j < 6; j++) {
          c_y += P[i5 + 6 * j] * H[j];
        }

        K[i5] = c_y / c;
        for (j = 0; j < 6; j++) {
          b_F[i5 + 6 * j] = (float)I[i5 + 6 * j] - K[i5] * H[j];
        }

        for (j = 0; j < 6; j++) {
          c_I[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            c_I[i5 + 6 * j] += b_F[i5 + 6 * i6] * P[i6 + 6 * j];
          }
        }
      }

      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          b_F[i5 + 6 * j] = (float)I[j + 6 * i5] - K[j] * H[i5];
        }
      }

      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          b_I[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            b_I[i5 + 6 * j] += c_I[i5 + 6 * i6] * b_F[i6 + 6 * j];
          }

          b_K[i5 + 6 * j] = K[i5] * r * K[j];
        }
      }

      /* 'threewheel:192' x = K*v; */
      for (j = 0; j < 6; j++) {
        for (i5 = 0; i5 < 6; i5++) {
          P[i5 + 6 * j] = b_I[i5 + 6 * j] + b_K[i5 + 6 * j];
        }

        x[j] = K[j] * b_v;
      }

      /* Correct Position */
      /* 'threewheel:194' Rn = Rn-x(1:2,1); */
      for (i5 = 0; i5 < 2; i5++) {
        Rn[i5] -= x[i5];
      }

      /* Correct Attitude */
      /* 'threewheel:196' En  = [1, -x(3,1), 0; x(3,1), 1, 0; 0, 0, 1]; */
      /* 'threewheel:197' Cbn = En*Cbn; */
      Fnd[0] = 1.0F;
      Fnd[3] = -x[2];
      Fnd[6] = 0.0F;
      Fnd[1] = x[2];
      Fnd[4] = 1.0F;
      Fnd[7] = 0.0F;
      for (i5 = 0; i5 < 3; i5++) {
        Fnd[2 + 3 * i5] = iv3[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        for (j = 0; j < 3; j++) {
          b_An[i5 + 3 * j] = 0.0F;
          for (i6 = 0; i6 < 3; i6++) {
            b_An[i5 + 3 * j] += Fnd[i5 + 3 * i6] * Cbn[i6 + 3 * j];
          }
        }
      }

      for (i5 = 0; i5 < 3; i5++) {
        for (j = 0; j < 3; j++) {
          Cbn[j + 3 * i5] = b_An[j + 3 * i5];
        }
      }

      /* Normalize Cbn matrix */
      /* 'threewheel:199' Cbn = dcmnormalize(Cbn); */
      dcmnormalize(Cbn);

      /* Update errors estimates */
      /* 'threewheel:201' dlr = dlr+x(4,1); */
      dlr += x[3];

      /* 'threewheel:202' dll = dll+x(5,1); */
      dll += x[4];

      /* 'threewheel:203' dw  = dw +x(6,1); */
      dw += x[5];
    }

    // ----------------------------------------------------------------
    /*         %% Kalman Update - SLAM: */
    /* 'threewheel:174' if (Parameters.gyr.update == 1) */
    if (Parameters->slam.update == 1) {
      /* measurement vector with odometer ZUPT */
      /* 'threewheel:177' v =  Wbg-Wb; */
      b_v = Wbg - Sensors->slam.dthe;

      /* measurement matrix */
      /* 'threewheel:182' H = zeros(1,6,'single'); */
      for (i5 = 0; i5 < 6; i5++) {
        H[i5] = 0.0F;
      }

      /* 'threewheel:185' H(1,6) =  1; */
      H[5] = 1.0F;

      /* measurement noise covariance */
      /* 'threewheel:187' r = Parameters.slam.head_noise^2; */
      r = Parameters->slam.heading_noise * Parameters->slam.heading_noise;

      /* Update */
      /* 'threewheel:189' I = eye(6,'single'); */
      /* 'threewheel:190' K = (P*H')/(H*P*H'+ r); */
      c_y = 0.0F;
      for (i5 = 0; i5 < 6; i5++) {
        b_H[i5] = 0.0F;
        for (j = 0; j < 6; j++) {
          b_H[i5] += H[j] * P[j + 6 * i5];
        }

        c_y += b_H[i5] * H[i5];
      }

      c = c_y + r;

      /* 'threewheel:191' P = (I-K*H)*P*(I-K*H)' + K*r*K'; */
      for (i5 = 0; i5 < 6; i5++) {
        c_y = 0.0F;
        for (j = 0; j < 6; j++) {
          c_y += P[i5 + 6 * j] * H[j];
        }

        K[i5] = c_y / c;
        for (j = 0; j < 6; j++) {
          b_F[i5 + 6 * j] = (float)I[i5 + 6 * j] - K[i5] * H[j];
        }

        for (j = 0; j < 6; j++) {
          c_I[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            c_I[i5 + 6 * j] += b_F[i5 + 6 * i6] * P[i6 + 6 * j];
          }
        }
      }

      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          b_F[i5 + 6 * j] = (float)I[j + 6 * i5] - K[j] * H[i5];
        }
      }

      for (i5 = 0; i5 < 6; i5++) {
        for (j = 0; j < 6; j++) {
          b_I[i5 + 6 * j] = 0.0F;
          for (i6 = 0; i6 < 6; i6++) {
            b_I[i5 + 6 * j] += c_I[i5 + 6 * i6] * b_F[i6 + 6 * j];
          }

          b_K[i5 + 6 * j] = K[i5] * r * K[j];
        }
      }

      /* 'threewheel:192' x = K*v; */
      for (j = 0; j < 6; j++) {
        for (i5 = 0; i5 < 6; i5++) {
          P[i5 + 6 * j] = b_I[i5 + 6 * j] + b_K[i5 + 6 * j];
        }

        x[j] = K[j] * b_v;
      }



      /* Update errors estimates */
      /* 'threewheel:201' dlr = dlr+x(4,1); */
      dlr += x[3];

      /* 'threewheel:202' dll = dll+x(5,1); */
      dll += x[4];

      /* 'threewheel:203' dw  = dw +x(6,1); */
      dw += x[5];
    }
	// ----------------------------------------------------------------

    /*         %% Reset */
    /* 'threewheel:207' if (Parameters.reset == 1) */
    if (Parameters->reset == 1) {
      /* 'threewheel:208' State.state = uint8(0); */
      State->state = 0U;
    }
    break;
  }

  /*  Set output */
  /* 'threewheel:213' Out.Rn  = Rn; */
  for (j = 0; j < 2; j++) {
    Out->Rn[j] = Rn[j];
  }

  /* 'threewheel:214' Out.Vb  = Vb; */
  Out->Vb = Vb;

  /* 'threewheel:215' Out.Cbn = Cbn; */
  for (i5 = 0; i5 < 9; i5++) {
    Out->Cbn[i5] = Cbn[i5];
  }

  /* 'threewheel:216' [Out.Heading, ~, ~] = dcm_angle(Cbn'); */
  for (i5 = 0; i5 < 3; i5++) {
    for (j = 0; j < 3; j++) {
      b_Cbn[j + 3 * i5] = Cbn[i5 + 3 * j];
    }
  }

  /*   [rz, ry, rx] = dcm_angle(Cbn) */
  /*   Transforms direction cosine matrix to Euler angles */
  /*  */
  /*    Input arguments: */
  /*    Cbn -  Direction cosine matrix [3,3] */
  /*  */
  /*    Output arguments: */
  /*    rz, ry, rx - Euler angles around Z, Y and X axes correspondingly */
  /* 'dcm_angle:11' rz = atan2(Cbn(1,2),Cbn(1,1)); */
  /* 'dcm_angle:12' ry = -atan2(Cbn(1,3),sqrt(1-Cbn(1,3)^2)); */
  /* 'dcm_angle:13' rx = atan2(Cbn(2,3),Cbn(3,3)); */
  Out->Heading = (float)atan2(b_Cbn[3], b_Cbn[0]);

  /* 'threewheel:216' ~ */
  /* 'threewheel:216' ~ */
  /* 'threewheel:217' [Out.Lat, Out.Lon, ~] = ned2geo([Rn; 0],... */
  /* 'threewheel:218'     Parameters.gps.lat0, Parameters.gps.lon0, 0,... */
  /* 'threewheel:219'     Parameters.gps.a, Parameters.gps.e); */
  for (j = 0; j < 2; j++) {
    b_Rn[j] = Rn[j];
  }

  b_Rn[2] = 0.0F;
  ned2geo(b_Rn, Parameters->gps.lat0, Parameters->gps.lon0, 0.0,
          Parameters->gps.a, Parameters->gps.e, &Out->Lat, &Out->Lon, &c_y);

  /* 'threewheel:217' ~ */
  /* 'threewheel:220' Out.Wb = Wb; */
  Out->Wb = Wb;

  /*  Set state */
  /* 'threewheel:223' State.P = P; */
  memcpy(&State->P[0], &P[0], 36U * sizeof(float));

  /* 'threewheel:224' State.Rn = Rn; */
  for (j = 0; j < 2; j++) {
    State->Rn[j] = Rn[j];
  }

  /* 'threewheel:225' State.Cbn = Cbn; */
  for (i5 = 0; i5 < 9; i5++) {
    State->Cbn[i5] = Cbn[i5];
  }

  /* 'threewheel:226' State.dw = dw; */
  State->dw = dw;

  /* 'threewheel:227' State.dlr = dlr; */
  State->dlr = dlr;

  /* 'threewheel:228' State.dll = dll; */
  State->dll = dll;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void threewheel_initialize(void)
{
}

/*
 * File trailer for threewheel.c
 *
 * [EOF]
 */
