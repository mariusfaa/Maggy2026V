//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: maglevModel.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 29-Jan-2026 14:32:41
//

// Include Files
#include "maglevModel.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Type Definitions
struct struct_T {
  double x[4];
  double y[4];
};

struct b_struct_T {
  double b_I[3];
};

struct c_struct_T {
  struct_T solenoids;
  struct_T permanent;
  b_struct_T magnet;
};

// Variable Definitions
static c_struct_T params;

static c_struct_T b_params;

static bool isInitialized_maglevModel{false};

// Function Declarations
namespace coder {
static void b_abs(const double x_data[], double y_data[], int y_size[2]);

static double cart2pol(double x, double y, double &r);

static void cross(const double a_data[], const double b_data[], double c_data[],
                  int c_size[2]);

static void diag(const double v[3], double d[9]);

static void ellipke(const double m_data[], double k_data[], int k_size[2],
                    double e_data[], int e_size[2]);

namespace internal {
namespace scalar {
static double b_atan2(double y, double x);

}
} // namespace internal
static void inv(const double x[9], double y[9]);

static void linspace(double y_data[], int y_size[2]);

static void mldivide(const double A[36], double B[6]);

static void pol2cart(const double th_data[], const double r_data[],
                     double x_data[], int x_size[2], double y_data[],
                     int y_size[2]);

static double scalar_ellipke(double m, double &e);

static double trapz(const double x_data[], const double y_data[]);

} // namespace coder
static void computeFieldCircularWireCartesian(
    const double x_data[], const double y_data[], const double z_data[],
    double r, double b_I, double bx_data[], int bx_size[2], double by_data[],
    int by_size[2], double bz_data[], int bz_size[2]);

static double computeFieldCircularWireCartesian(double x, double y, double z,
                                                double r, double b_I,
                                                double &by, double &bz);

static void maglevSystemDynamics_fast_init();

static void maglevSystemMeasurements_fast_init();

static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : const double x_data[]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
namespace coder {
static void b_abs(const double x_data[], double y_data[], int y_size[2])
{
  y_size[0] = 1;
  y_size[1] = 100;
  for (int k{0}; k < 100; k++) {
    y_data[k] = std::abs(x_data[k]);
  }
}

//
// Arguments    : double x
//                double y
//                double &r
// Return Type  : double
//
static double cart2pol(double x, double y, double &r)
{
  double a;
  double b;
  double th;
  if (std::isnan(y) || std::isnan(x)) {
    th = rtNaN;
  } else if (std::isinf(y) && std::isinf(x)) {
    int i;
    int i1;
    if (y > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (x > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    th = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (x == 0.0) {
    if (y > 0.0) {
      th = RT_PI / 2.0;
    } else if (y < 0.0) {
      th = -(RT_PI / 2.0);
    } else {
      th = 0.0;
    }
  } else {
    th = std::atan2(y, x);
  }
  a = std::abs(x);
  b = std::abs(y);
  if (a < b) {
    a /= b;
    r = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    r = a * std::sqrt(b * b + 1.0);
  } else if (std::isnan(b)) {
    r = rtNaN;
  } else {
    r = a * 1.4142135623730951;
  }
  return th;
}

//
// Arguments    : const double a_data[]
//                const double b_data[]
//                double c_data[]
//                int c_size[2]
// Return Type  : void
//
static void cross(const double a_data[], const double b_data[], double c_data[],
                  int c_size[2])
{
  int dim;
  int iNext;
  int nHigh;
  int stride;
  int stridem1;
  bool exitg1;
  c_size[0] = 3;
  c_size[1] = 100;
  dim = 0;
  nHigh = 0;
  exitg1 = false;
  while ((!exitg1) && (nHigh < 2)) {
    if (97 * nHigh == 0) {
      dim = nHigh + 1;
      exitg1 = true;
    } else {
      nHigh++;
    }
  }
  if (dim >= 2) {
    stride = 3;
    stridem1 = 2;
  } else {
    stride = 1;
    stridem1 = 0;
  }
  iNext = stride * 3;
  if (dim >= 2) {
    nHigh = 1;
  } else {
    nHigh = iNext * 99 + 1;
  }
  for (int iStart{1}; iNext < 0 ? iStart >= nHigh : iStart <= nHigh;
       iStart += iNext) {
    dim = iStart + stridem1;
    for (int i1{iStart}; i1 <= dim; i1++) {
      double d;
      double d1;
      int i2;
      int i3;
      i2 = (i1 + stride) - 1;
      i3 = i2 + stride;
      c_data[i1 - 1] = a_data[i2] * b_data[i3] - a_data[i3] * b_data[i2];
      d = a_data[i1 - 1];
      d1 = b_data[i1 - 1];
      c_data[i2] = a_data[i3] * d1 - d * b_data[i3];
      c_data[i3] = d * b_data[i2] - a_data[i2] * d1;
    }
  }
}

//
// Arguments    : const double v[3]
//                double d[9]
// Return Type  : void
//
static void diag(const double v[3], double d[9])
{
  std::memset(&d[0], 0, 9U * sizeof(double));
  d[0] = v[0];
  d[4] = v[1];
  d[8] = v[2];
}

//
// Arguments    : const double m_data[]
//                double k_data[]
//                int k_size[2]
//                double e_data[]
//                int e_size[2]
// Return Type  : void
//
static void ellipke(const double m_data[], double k_data[], int k_size[2],
                    double e_data[], int e_size[2])
{
  k_size[0] = 1;
  k_size[1] = 100;
  e_size[0] = 1;
  e_size[1] = 100;
  for (int k{0}; k < 100; k++) {
    k_data[k] = scalar_ellipke(m_data[k], e_data[k]);
  }
}

//
// Arguments    : double y
//                double x
// Return Type  : double
//
namespace internal {
namespace scalar {
static double b_atan2(double y, double x)
{
  double r;
  if (std::isnan(y) || std::isnan(x)) {
    r = rtNaN;
  } else if (std::isinf(y) && std::isinf(x)) {
    int i;
    int i1;
    if (y > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (x > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    r = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (x == 0.0) {
    if (y > 0.0) {
      r = RT_PI / 2.0;
    } else if (y < 0.0) {
      r = -(RT_PI / 2.0);
    } else {
      r = 0.0;
    }
  } else {
    r = std::atan2(y, x);
  }
  return r;
}

//
// Arguments    : const double x[9]
//                double y[9]
// Return Type  : void
//
} // namespace scalar
} // namespace internal
static void inv(const double x[9], double y[9])
{
  double b_x[9];
  double absx11;
  double absx21;
  double absx31;
  int p1;
  int p2;
  int p3;
  std::copy(&x[0], &x[9], &b_x[0]);
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = std::abs(x[0]);
  absx21 = std::abs(x[1]);
  absx31 = std::abs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    b_x[0] = x[2];
    b_x[2] = x[0];
    b_x[3] = x[5];
    b_x[5] = x[3];
    b_x[6] = x[8];
    b_x[8] = x[6];
  }
  b_x[1] /= b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= b_x[1] * b_x[3];
  b_x[5] -= b_x[2] * b_x[3];
  b_x[7] -= b_x[1] * b_x[6];
  b_x[8] -= b_x[2] * b_x[6];
  if (std::abs(b_x[5]) > std::abs(b_x[4])) {
    int itmp;
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = b_x[1];
    b_x[1] = b_x[2];
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }
  b_x[5] /= b_x[4];
  b_x[8] -= b_x[5] * b_x[7];
  absx11 = (b_x[1] * b_x[5] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

//
// Arguments    : double y_data[]
//                int y_size[2]
// Return Type  : void
//
static void linspace(double y_data[], int y_size[2])
{
  y_size[0] = 1;
  y_size[1] = 100;
  y_data[99] = 6.2203534541077907;
  y_data[0] = 0.0;
  for (int k{0}; k < 98; k++) {
    y_data[k + 1] = (static_cast<double>(k) + 1.0) * 0.062831853071795868;
  }
}

//
// Arguments    : const double A[36]
//                double B[6]
// Return Type  : void
//
static void mldivide(const double A[36], double B[6])
{
  double b_A[36];
  double smax;
  int a;
  int jA;
  signed char ipiv[6];
  std::copy(&A[0], &A[36], &b_A[0]);
  for (int k{0}; k < 6; k++) {
    ipiv[k] = static_cast<signed char>(k + 1);
  }
  for (int j{0}; j < 5; j++) {
    int A_tmp;
    int b;
    int jj;
    int jp1j;
    int mmj;
    signed char i;
    mmj = 4 - j;
    b = j * 7;
    jj = j * 7;
    jp1j = b + 2;
    jA = 7 - j;
    a = 0;
    smax = std::abs(b_A[jj]);
    for (int b_k{2}; b_k < jA; b_k++) {
      double s;
      s = std::abs(b_A[(b + b_k) - 1]);
      if (s > smax) {
        a = b_k - 1;
        smax = s;
      }
    }
    if (b_A[jj + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = static_cast<signed char>(jA + 1);
        for (int k{0}; k < 6; k++) {
          a = j + k * 6;
          smax = b_A[a];
          A_tmp = jA + k * 6;
          b_A[a] = b_A[A_tmp];
          b_A[A_tmp] = smax;
        }
      }
      jA = (jj - j) + 6;
      for (int k{jp1j}; k <= jA; k++) {
        b_A[k - 1] /= b_A[jj];
      }
    }
    jA = jj;
    for (int b_k{0}; b_k <= mmj; b_k++) {
      smax = b_A[(b + b_k * 6) + 6];
      if (smax != 0.0) {
        a = jA + 8;
        A_tmp = (jA - j) + 12;
        for (int k{a}; k <= A_tmp; k++) {
          b_A[k - 1] += b_A[((jj + k) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
    i = ipiv[j];
    if (i != j + 1) {
      smax = B[j];
      B[j] = B[i - 1];
      B[i - 1] = smax;
    }
  }
  for (int k{0}; k < 6; k++) {
    jA = 6 * k;
    if (B[k] != 0.0) {
      a = k + 2;
      for (int b_k{a}; b_k < 7; b_k++) {
        B[b_k - 1] -= B[k] * b_A[(b_k + jA) - 1];
      }
    }
  }
  for (int k{5}; k >= 0; k--) {
    jA = 6 * k;
    smax = B[k];
    if (smax != 0.0) {
      smax /= b_A[k + jA];
      B[k] = smax;
      for (int b_k{0}; b_k < k; b_k++) {
        B[b_k] -= B[k] * b_A[b_k + jA];
      }
    }
  }
}

//
// Arguments    : const double th_data[]
//                const double r_data[]
//                double x_data[]
//                int x_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
static void pol2cart(const double th_data[], const double r_data[],
                     double x_data[], int x_size[2], double y_data[],
                     int y_size[2])
{
  x_size[0] = 1;
  x_size[1] = 100;
  y_size[0] = 1;
  y_size[1] = 100;
  for (int k{0}; k < 100; k++) {
    double d;
    double d1;
    d = th_data[k];
    d1 = r_data[k];
    x_data[k] = d1 * std::cos(d);
    y_data[k] = d1 * std::sin(d);
  }
}

//
// Arguments    : double m
//                double &e
// Return Type  : double
//
static double scalar_ellipke(double m, double &e)
{
  double k;
  if (m == 1.0) {
    k = rtInf;
    e = 1.0;
  } else {
    double a1;
    double b0;
    double i1;
    double s0;
    double w1;
    k = 1.0;
    b0 = std::sqrt(1.0 - m);
    s0 = m;
    i1 = 0.0;
    w1 = 1.0;
    a1 = 1.0;
    while (w1 > 2.2204460492503131E-16) {
      double b1;
      a1 = (k + b0) / 2.0;
      b1 = std::sqrt(k * b0);
      k = (k - b0) / 2.0;
      i1++;
      w1 = rt_powd_snf(2.0, i1) * k * k;
      s0 += w1;
      k = a1;
      b0 = b1;
    }
    k = 3.1415926535897931 / (2.0 * a1);
    e = k * (1.0 - s0 / 2.0);
  }
  return k;
}

//
// Arguments    : const double x_data[]
//                const double y_data[]
// Return Type  : double
//
static double trapz(const double x_data[], const double y_data[])
{
  double c_data[101];
  double z;
  int ix;
  c_data[0] = 0.5 * (x_data[1] - x_data[0]);
  for (int k{0}; k < 99; k++) {
    c_data[k + 1] = 0.5 * (x_data[k + 2] - x_data[k]);
  }
  c_data[100] = 0.5 * (x_data[100] - x_data[99]);
  z = 0.0;
  ix = 0;
  for (int k{0}; k < 101; k++) {
    for (int ia{k + 1}; ia <= k + 1; ia++) {
      z += y_data[ia - 1] * c_data[ix];
    }
    ix++;
  }
  return z;
}

//
// COMPUTEFIELDCIRCULARWIRECARTESIAN computes the magnetic field in
//  cartesian coordinates produced by a single circular current carrying wire.
//  The wire is centered at the origin and lies parallel to the xy-plane.
//
//  The function calculates the magnetic field components (bx, by, bz) at
//  the specified points (x, y, z). The strength and direction of the
//  magnetic field is determined by the radius r of the wire loop, the
//  current I running through it and the magnetic permeability of the medium.
//
//  Example:
//    x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
//    r = 1; I = 1; mu0 = 4*pi*1e-7;
//    [bphi, brho, bz] = computeFieldCircularWireCartesian(x,y,z,r,I,mu0);
//
//  See also COMPUTEFIELDCIRCULARWIREPOLAR,
//           COMPUTEFIELDCIRCULARCURRENTSHEETCARTESIAN.
//
// Arguments    : double x
//                double y
//                double z
//                double r
//                double b_I
//                double &by
//                double &bz
// Return Type  : double
//
} // namespace coder
static double computeFieldCircularWireCartesian(double x, double y, double z,
                                                double r, double b_I,
                                                double &by, double &bz)
{
  double E;
  double K;
  double a;
  double bx;
  double c;
  double d;
  double d1;
  double d2;
  double d3;
  double k2_tmp;
  double rho;
  //  COMPUTEFIELDCIRCULARWIRECARTESIAN uses the implementation described in
  //  [1]. Author: Hans Alvar Engmark Date: 08.01.2024 References: [1] Engmark,
  //  Hans Alvar, and Kiet Tuan Hoang.
  //      "Modeling and Control of a Magnetic Levitation Platform."
  //      IFAC-PapersOnLine 56.2 (2023): 7276-7281.
  //  Convert input to polar coordinates
  d = coder::cart2pol(x, y, rho);
  //  Compute field
  //  COMPUTEFIELDCIRCULARWIREPOLAR computes the magnetic field in polar
  //  coordinates produced by a single circular current carrying wire.
  //  The wire is centered at the origin and lies parallel to the xy-plane.
  //  The function calculates the magnetic field components (bphi, brho, bz) at
  //  the specified points (phi, rho, z) in polar coordinates. The strength and
  //  direction of the magnetic field is determined by the radius r of the wire
  //  loop, the current I running through it and the magnetic permeability of
  //  the medium (e.g., air).
  //  Example:
  //    phi = [0, pi/4, pi/2]; rho = [1, 1.5, 2]; z = [0, 0.5, 1];
  //    r = 1; I = 1; mu0 = 4*pi*1e-7;
  //    [bphi, brho, bz] = computeFieldCircularWirePolar(phi,rho,z,r,I,mu0);
  //  See also COMPUTEFIELDCIRCULARWIRECARTESIAN,
  //           COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR.
  //  COMPUTEFIELDCIRCULARWIREPOLAR uses the implementation described in [1].
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  References:
  //  [1] Engmark, Hans Alvar, and Kiet Tuan Hoang.
  //      "Modeling and Control of a Magnetic Levitation Platform."
  //      IFAC-PapersOnLine 56.2 (2023): 7276-7281.
  //  rho ~= 0 (2a)
  c = 1.2566370614359173E-6 * b_I / (12.566370614359172 * std::sqrt(r * rho));
  bx = r + rho;
  k2_tmp = z * z;
  bx = std::fmin(4.0 * r * rho / (bx * bx + k2_tmp), 1.0);
  //  Fix for numerical error
  K = coder::scalar_ellipke(bx, E);
  a = rho - r;
  d1 = std::sqrt(bx);
  d2 = rho * rho;
  d3 = r * r;
  bx = a * a + k2_tmp;
  a = -(z / rho) * c * d1 * (K - ((d2 + d3) + k2_tmp) / bx * E);
  bx = c * d1 * (K - ((d2 - d3) + k2_tmp) / bx * E);
  //  rho = 0 (2b)
  //  Fix for numerical error
  if (std::abs(rho) < 1.0E-6) {
    d = 0.0;
    a = 0.0;
    bx = 1.2566370614359173E-6 * d3 * b_I /
         (2.0 * rt_powd_snf(d3 + k2_tmp, 1.5));
  }
  bz = bx;
  //  Convert result to Cartesian coordinates
  bx = a * std::cos(d);
  by = a * std::sin(d);
  return bx;
}

//
// COMPUTEFIELDCIRCULARWIRECARTESIAN computes the magnetic field in
//  cartesian coordinates produced by a single circular current carrying wire.
//  The wire is centered at the origin and lies parallel to the xy-plane.
//
//  The function calculates the magnetic field components (bx, by, bz) at
//  the specified points (x, y, z). The strength and direction of the
//  magnetic field is determined by the radius r of the wire loop, the
//  current I running through it and the magnetic permeability of the medium.
//
//  Example:
//    x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
//    r = 1; I = 1; mu0 = 4*pi*1e-7;
//    [bphi, brho, bz] = computeFieldCircularWireCartesian(x,y,z,r,I,mu0);
//
//  See also COMPUTEFIELDCIRCULARWIREPOLAR,
//           COMPUTEFIELDCIRCULARCURRENTSHEETCARTESIAN.
//
// Arguments    : const double x_data[]
//                const double y_data[]
//                const double z_data[]
//                double r
//                double b_I
//                double bx_data[]
//                int bx_size[2]
//                double by_data[]
//                int by_size[2]
//                double bz_data[]
//                int bz_size[2]
// Return Type  : void
//
static void computeFieldCircularWireCartesian(
    const double x_data[], const double y_data[], const double z_data[],
    double r, double b_I, double bx_data[], int bx_size[2], double by_data[],
    int by_size[2], double bz_data[], int bz_size[2])
{
  double E_data[100];
  double K_data[100];
  double b_x_data[100];
  double b_y_data[100];
  double k2_data[100];
  double phi_data[100];
  double rho_data[100];
  double a;
  double c;
  double d;
  double varargin_1;
  double varargin_2;
  double x;
  int K_size[2];
  int partialTrueCount;
  int trueCount;
  signed char tmp_data[100];
  //  COMPUTEFIELDCIRCULARWIRECARTESIAN uses the implementation described in
  //  [1]. Author: Hans Alvar Engmark Date: 08.01.2024 References: [1] Engmark,
  //  Hans Alvar, and Kiet Tuan Hoang.
  //      "Modeling and Control of a Magnetic Levitation Platform."
  //      IFAC-PapersOnLine 56.2 (2023): 7276-7281.
  //  Convert input to polar coordinates
  //  Compute field
  //  COMPUTEFIELDCIRCULARWIREPOLAR computes the magnetic field in polar
  //  coordinates produced by a single circular current carrying wire.
  //  The wire is centered at the origin and lies parallel to the xy-plane.
  //  The function calculates the magnetic field components (bphi, brho, bz) at
  //  the specified points (phi, rho, z) in polar coordinates. The strength and
  //  direction of the magnetic field is determined by the radius r of the wire
  //  loop, the current I running through it and the magnetic permeability of
  //  the medium (e.g., air).
  //  Example:
  //    phi = [0, pi/4, pi/2]; rho = [1, 1.5, 2]; z = [0, 0.5, 1];
  //    r = 1; I = 1; mu0 = 4*pi*1e-7;
  //    [bphi, brho, bz] = computeFieldCircularWirePolar(phi,rho,z,r,I,mu0);
  //  See also COMPUTEFIELDCIRCULARWIRECARTESIAN,
  //           COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR.
  //  COMPUTEFIELDCIRCULARWIREPOLAR uses the implementation described in [1].
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  References:
  //  [1] Engmark, Hans Alvar, and Kiet Tuan Hoang.
  //      "Modeling and Control of a Magnetic Levitation Platform."
  //      IFAC-PapersOnLine 56.2 (2023): 7276-7281.
  //  rho ~= 0 (2a)
  x = 1.2566370614359173E-6 * b_I;
  a = 4.0 * r;
  for (int k{0}; k < 100; k++) {
    varargin_1 = y_data[k];
    varargin_2 = x_data[k];
    phi_data[k] = coder::internal::scalar::b_atan2(varargin_1, varargin_2);
    varargin_2 = std::abs(varargin_2);
    varargin_1 = std::abs(varargin_1);
    if (varargin_2 < varargin_1) {
      varargin_2 /= varargin_1;
      d = varargin_1 * std::sqrt(varargin_2 * varargin_2 + 1.0);
    } else if (varargin_2 > varargin_1) {
      varargin_1 /= varargin_2;
      d = varargin_2 * std::sqrt(varargin_1 * varargin_1 + 1.0);
    } else if (std::isnan(varargin_1)) {
      d = rtNaN;
    } else {
      d = varargin_2 * 1.4142135623730951;
    }
    rho_data[k] = d;
    varargin_2 = z_data[k];
    varargin_2 *= varargin_2;
    b_y_data[k] = varargin_2;
    varargin_1 = r + d;
    k2_data[k] = std::fmin(a * d / (varargin_1 * varargin_1 + varargin_2), 1.0);
  }
  int E_size[2];
  //  Fix for numerical error
  coder::ellipke(k2_data, K_data, K_size, E_data, E_size);
  c = r * r;
  bz_size[0] = 1;
  bz_size[1] = 100;
  for (int k{0}; k < 100; k++) {
    double b_varargin_1;
    double d1;
    double d2;
    double varargout_1;
    d1 = std::sqrt(k2_data[k]);
    k2_data[k] = d1;
    b_varargin_1 = rho_data[k];
    varargout_1 = b_varargin_1 * b_varargin_1;
    varargin_2 = b_varargin_1 - r;
    d2 = b_y_data[k];
    varargin_2 = varargin_2 * varargin_2 + d2;
    varargin_1 = K_data[k];
    d = E_data[k];
    a = x / (12.566370614359172 * std::sqrt(r * b_varargin_1));
    b_x_data[k] = -(z_data[k] / b_varargin_1) * a * d1 *
                  (varargin_1 - ((varargout_1 + c) + d2) / varargin_2 * d);
    bz_data[k] =
        a * d1 * (varargin_1 - ((varargout_1 - c) + d2) / varargin_2 * d);
  }
  //  rho = 0 (2b)
  //  Fix for numerical error
  coder::b_abs(rho_data, k2_data, K_size);
  trueCount = 0;
  partialTrueCount = 0;
  for (int k{0}; k < 100; k++) {
    if (k2_data[k] < 1.0E-6) {
      trueCount++;
      tmp_data[partialTrueCount] = static_cast<signed char>(k);
      partialTrueCount++;
    }
  }
  varargin_2 = 1.2566370614359173E-6 * c * b_I;
  for (int k{0}; k < trueCount; k++) {
    signed char i;
    i = tmp_data[k];
    phi_data[i] = 0.0;
    b_x_data[i] = 0.0;
    varargin_1 = z_data[i];
    bz_data[i] =
        varargin_2 / (2.0 * rt_powd_snf(c + varargin_1 * varargin_1, 1.5));
  }
  //  Convert result to Cartesian coordinates
  coder::pol2cart(phi_data, b_x_data, bx_data, bx_size, by_data, by_size);
}

//
// Arguments    : void
// Return Type  : void
//
static void maglevSystemDynamics_fast_init()
{
  //     %% Parameters
  //  Solenoids (Tuned to real solenoids and data from gikfun)
  //  mod; 'fast' parameter correction 0.5644
  //  % Permanent magnets (Tuned to the real magnets)
  params.solenoids.x[0] = 0.02;
  params.solenoids.y[0] = 0.0;
  params.permanent.x[0] = 0.024748737341529166;
  params.permanent.y[0] = 0.024748737341529166;
  params.solenoids.x[1] = 0.0;
  params.solenoids.y[1] = 0.02;
  params.permanent.x[1] = -0.024748737341529166;
  params.permanent.y[1] = 0.024748737341529166;
  params.solenoids.x[2] = -0.02;
  params.solenoids.y[2] = 0.0;
  params.permanent.x[2] = 0.024748737341529166;
  params.permanent.y[2] = -0.024748737341529166;
  params.solenoids.x[3] = 0.0;
  params.solenoids.y[3] = -0.02;
  params.permanent.x[3] = -0.024748737341529166;
  params.permanent.y[3] = -0.024748737341529166;
  //  (where they are centered)
  //  Levitating magnet (Tuned using the "equivalent magnet" principle)
  //  (weight on kitchen scale, golden magnet)
  params.magnet.b_I[0] = 6.1686E-6;
  params.magnet.b_I[1] = 6.1686E-6;
  params.magnet.b_I[2] = 1.1274E-5;
  //  Sensors (7, 2, 3)
  // , -0.0326856, 0.0130152];
  // , 0.0137257, 0.0324254];
  // , 0, 0];%-0.2e-3;
  //  Physical constants
  //  Gravitational acceleration [m/s^2]
}

//
// Arguments    : void
// Return Type  : void
//
static void maglevSystemMeasurements_fast_init()
{
  //     %% Parameters
  //  Solenoids (Tuned to real solenoids and data from gikfun)
  //  mod; 'fast' parameter correction 0.5644
  //  % Permanent magnets (Tuned to the real magnets)
  b_params.solenoids.x[0] = 0.02;
  b_params.solenoids.y[0] = 0.0;
  b_params.permanent.x[0] = 0.024748737341529166;
  b_params.permanent.y[0] = 0.024748737341529166;
  b_params.solenoids.x[1] = 0.0;
  b_params.solenoids.y[1] = 0.02;
  b_params.permanent.x[1] = -0.024748737341529166;
  b_params.permanent.y[1] = 0.024748737341529166;
  b_params.solenoids.x[2] = -0.02;
  b_params.solenoids.y[2] = 0.0;
  b_params.permanent.x[2] = 0.024748737341529166;
  b_params.permanent.y[2] = -0.024748737341529166;
  b_params.solenoids.x[3] = 0.0;
  b_params.solenoids.y[3] = -0.02;
  b_params.permanent.x[3] = -0.024748737341529166;
  b_params.permanent.y[3] = -0.024748737341529166;
  //  (where they are centered)
  //  Levitating magnet (Tuned using the "equivalent magnet" principle)
  //  (weight on kitchen scale, golden magnet)
  b_params.magnet.b_I[0] = 6.1686E-6;
  b_params.magnet.b_I[1] = 6.1686E-6;
  b_params.magnet.b_I[2] = 1.1274E-5;
  //  Sensors (7, 2, 3)
  // , -0.0326856, 0.0130152];
  // , 0.0137257, 0.0324254];
  // , 0, 0];%-0.2e-3;
  //  Physical constants
  //  Gravitational acceleration [m/s^2]
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    y = std::abs(u0);
    d = std::abs(u1);
    if (std::isinf(u1)) {
      if (y == 1.0) {
        y = 1.0;
      } else if (y > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d == 0.0) {
      y = 1.0;
    } else if (d == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : void
// Return Type  : void
//
void maglevModel_initialize()
{
  maglevSystemDynamics_fast_init();
  maglevSystemMeasurements_fast_init();
  isInitialized_maglevModel = true;
}

//
// Arguments    : void
// Return Type  : void
//
void maglevModel_terminate()
{
  isInitialized_maglevModel = false;
}

//
// Arguments    : const double x[12]
//                const double u[4]
//                double dx[12]
// Return Type  : void
//
void maglevSystemDynamics_fast(const double x[12], const double u[4],
                               double dx[12])
{
  static const double dv3[18]{0.06, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.06,
                              0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0};
  static const double dv4[6]{0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  static const signed char b_a[144]{
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  static const signed char c_a[72]{
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[3]{1, 0, 0};
  double T_data[300];
  double c_x_data[300];
  double p_data[300];
  double tmp_data[300];
  double b_T_data[101];
  double b_theta_data[101];
  double c_T_data[101];
  double c_theta_data[101];
  double d_T_data[101];
  double d_theta_data[101];
  double e_p_data[101];
  double e_theta_data[101];
  double f_p_data[101];
  double f_theta_data[101];
  double g_p_data[101];
  double g_theta_data[101];
  double b_p_data[100];
  double b_x_data[100];
  double bx_data[100];
  double by_data[100];
  double bzTemp_data[100];
  double bz_data[100];
  double c_p_data[100];
  double d_p_data[100];
  double theta_data[100];
  double x_data[100];
  double dv2[36];
  double a[12];
  double R[9];
  double c_Rz_tmp[9];
  double d_Rz_tmp[9];
  double dv[6];
  double dv1[6];
  double b[3];
  double Rx_tmp;
  double Ry_tmp;
  double Rz_tmp;
  double b_Rx_tmp;
  double b_Ry_tmp;
  double b_Rz_tmp;
  int bzTemp_size[2];
  int theta_size[2];
  int x_size[2];
  int Rz_tmp_tmp;
  int b_Rz_tmp_tmp;
  if (!isInitialized_maglevModel) {
    maglevModel_initialize();
  }
  //  MAGLEVSYSTEMDYNAMICS implements the function f in the ODE dxdt = f(x,u)
  //  defining the dynamics of a magnetic levitation system. The system is
  //  fully defined by the params struct, and the magnetic model to be used is
  //  defined by modelName, which can be either 'fast', 'accurate' or
  //  'filament'.
  //  Example:
  //    params; (from parameter file)
  //    modelName = 'fast';
  //    u = @(t) ...; (user defined)
  //    f = @(t,x) maglevSystemDynamics(x,u(t),params,modelName);
  //    t = [0, 10]; x0 = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
  //    [t,x] = ode15s(f,t,x0);
  //  See also MAGLEVSYSTEMMEASUREMENTS.
  //  MAGLEVSYSTEMDYNAMICS uses the implementation described in [1].
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  References:
  //  [1] Engmark, Hans Alvar, and Kiet Tuan Hoang.
  //      "Modeling and Control of a Magnetic Levitation Platform."
  //      IFAC-PapersOnLine 56.2 (2023): 7276-7281.
  //  Computing force and torque on levitating magnet
  //  COMPUTEFORCEANDTORQUE computes the magnetic force and torque produced by
  //  a base of permanent magnets and solenoids, defined by params,
  //  on a levitating magnet defined by params and x. The force is computed
  //  using the magnet/solenoid model defined by modelName.
  //  u is the current in running through the solenoids (its size defined by
  //  the number of solenoids in params). modelName is either 'fast',
  //  'accurate' or 'fillament'.
  //  Example:
  //    x = [0,0,0.05,0,0,0,0,0,0,0,0,0]'; u = [1,0,-1,0]';
  //    params; (from parameter file)
  //    modelName = 'fast';
  //    [fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName);
  //  See also COMPUTEFIELDBASE.
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  Additional parameters
  //  Default is 'fast'
  //  Points along circumfrence
  coder::linspace(theta_data, theta_size);
  for (int k{0}; k < 100; k++) {
    Ry_tmp = theta_data[k];
    x_data[k] = std::cos(Ry_tmp);
    b_x_data[k] = std::sin(Ry_tmp);
  }
  //  Euler rotation matrix
  Rx_tmp = std::sin(x[3]);
  b_Rx_tmp = std::cos(x[3]);
  Ry_tmp = std::sin(x[4]);
  b_Ry_tmp = std::cos(x[4]);
  Rz_tmp = std::sin(x[5]);
  b_Rz_tmp = std::cos(x[5]);
  c_Rz_tmp[0] = b_Rz_tmp;
  c_Rz_tmp[3] = -Rz_tmp;
  c_Rz_tmp[6] = 0.0;
  c_Rz_tmp[1] = Rz_tmp;
  c_Rz_tmp[4] = b_Rz_tmp;
  c_Rz_tmp[7] = 0.0;
  R[0] = b_Ry_tmp;
  R[3] = 0.0;
  R[6] = Ry_tmp;
  c_Rz_tmp[2] = 0.0;
  R[1] = 0.0;
  c_Rz_tmp[5] = 0.0;
  R[4] = 1.0;
  c_Rz_tmp[8] = 1.0;
  R[7] = 0.0;
  R[2] = -Ry_tmp;
  R[5] = 0.0;
  R[8] = b_Ry_tmp;
  std::memset(&d_Rz_tmp[0], 0, 9U * sizeof(double));
  for (int k{0}; k < 3; k++) {
    Ry_tmp = d_Rz_tmp[3 * k];
    Rz_tmp_tmp = 3 * k + 1;
    b_Rz_tmp_tmp = 3 * k + 2;
    for (int i{0}; i < 3; i++) {
      b_Ry_tmp = R[i + 3 * k];
      Ry_tmp += c_Rz_tmp[3 * i] * b_Ry_tmp;
      d_Rz_tmp[Rz_tmp_tmp] += c_Rz_tmp[3 * i + 1] * b_Ry_tmp;
      d_Rz_tmp[b_Rz_tmp_tmp] += c_Rz_tmp[3 * i + 2] * b_Ry_tmp;
    }
    d_Rz_tmp[3 * k] = Ry_tmp;
  }
  c_Rz_tmp[1] = 0.0;
  c_Rz_tmp[4] = b_Rx_tmp;
  c_Rz_tmp[7] = -Rx_tmp;
  c_Rz_tmp[2] = 0.0;
  c_Rz_tmp[5] = Rx_tmp;
  c_Rz_tmp[8] = b_Rx_tmp;
  std::memset(&R[0], 0, 9U * sizeof(double));
  for (int k{0}; k < 3; k++) {
    c_Rz_tmp[3 * k] = iv[k];
    Ry_tmp = R[3 * k];
    Rz_tmp_tmp = 3 * k + 1;
    b_Rz_tmp_tmp = 3 * k + 2;
    for (int i{0}; i < 3; i++) {
      b_Ry_tmp = c_Rz_tmp[i + 3 * k];
      Ry_tmp += d_Rz_tmp[3 * i] * b_Ry_tmp;
      R[Rz_tmp_tmp] += d_Rz_tmp[3 * i + 1] * b_Ry_tmp;
      R[b_Rz_tmp_tmp] += d_Rz_tmp[3 * i + 2] * b_Ry_tmp;
    }
    R[3 * k] = Ry_tmp;
  }
  //  Compute magnetic field
  //  COMPUTEFIELDBASE computes the magnetic field in cartesian coordinates
  //  produced by a base of permanent magnets and solenoids, defined by params,
  //  using the magnet/solenoid model defined by modelName.
  //  The function calculates the magnetic field components (bx, by, bz) at
  //  the specified points (x, y, z) in polar coordinates. u is the current in
  //  running through the solenoids (its size defined by the number of
  //  solenoids in params). modelName is either 'fast', 'accurate' or
  //  'fillament'.
  //  Example:
  //    x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
  //    u = [1,0,-1,0]';
  //    params; (from parameter file)
  //    modelName = 'fast';
  //    [bx,by,bz] = computeFieldBase(x,y,z,u,params,modelName);
  //  See also COMPUTEFIELDTOTAL,
  //           COMPUTEFORCEANDTORQUE.
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  Initialize field array
  for (int k{0}; k < 100; k++) {
    tmp_data[3 * k] = 0.025 * x_data[k];
    Rz_tmp_tmp = 3 * k + 1;
    tmp_data[Rz_tmp_tmp] = 0.025 * b_x_data[k];
    b_Rz_tmp_tmp = 3 * k + 2;
    tmp_data[b_Rz_tmp_tmp] = 0.0;
    Ry_tmp = 0.0;
    b_Ry_tmp = 0.0;
    Rz_tmp = 0.0;
    for (int i{0}; i < 3; i++) {
      b_Rz_tmp = tmp_data[i + 3 * k];
      Ry_tmp += R[3 * i] * b_Rz_tmp;
      b_Ry_tmp += R[3 * i + 1] * b_Rz_tmp;
      Rz_tmp += R[3 * i + 2] * b_Rz_tmp;
    }
    p_data[3 * k] = Ry_tmp + x[0];
    p_data[Rz_tmp_tmp] = b_Ry_tmp + x[1];
    p_data[b_Rz_tmp_tmp] = Rz_tmp + x[2];
    bx_data[k] = 0.0;
    by_data[k] = 0.0;
    bz_data[k] = 0.0;
  }
  //  Field from permanent magnets
  //  Default is 'fast'
  for (int i{0}; i < 4; i++) {
    Ry_tmp = params.permanent.x[i];
    b_Ry_tmp = params.permanent.y[i];
    for (int k{0}; k < 100; k++) {
      b_p_data[k] = p_data[3 * k] - Ry_tmp;
      c_p_data[k] = p_data[3 * k + 1] - b_Ry_tmp;
      d_p_data[k] = p_data[3 * k + 2] - 0.0042;
    }
    computeFieldCircularWireCartesian(
        b_p_data, c_p_data, d_p_data, 0.01, 7321.1273822271851, x_data,
        theta_size, b_x_data, x_size, bzTemp_data, bzTemp_size);
    for (int k{0}; k < 100; k++) {
      bx_data[k] += x_data[k];
      by_data[k] += b_x_data[k];
      bz_data[k] += bzTemp_data[k];
    }
  }
  //  Field from solenoids
  //  Default is 'fast'
  for (int i{0}; i < 4; i++) {
    Ry_tmp = params.solenoids.x[i];
    b_Ry_tmp = params.solenoids.y[i];
    for (int k{0}; k < 100; k++) {
      b_p_data[k] = p_data[3 * k] - Ry_tmp;
      c_p_data[k] = p_data[3 * k + 1] - b_Ry_tmp;
      d_p_data[k] = p_data[3 * k + 2] - 0.006;
    }
    computeFieldCircularWireCartesian(
        b_p_data, c_p_data, d_p_data, 0.00786622775, u[i], x_data, theta_size,
        b_x_data, x_size, bzTemp_data, bzTemp_size);
    for (int k{0}; k < 100; k++) {
      bx_data[k] += x_data[k] * 480.0;
      by_data[k] += b_x_data[k] * 480.0;
      bz_data[k] += bzTemp_data[k] * 480.0;
    }
  }
  //  Compute force
  for (int k{0}; k < 100; k++) {
    Ry_tmp = theta_data[k] + 1.5707963267948966;
    c_x_data[3 * k] = std::cos(Ry_tmp);
    Rz_tmp_tmp = 3 * k + 1;
    c_x_data[Rz_tmp_tmp] = std::sin(Ry_tmp);
    b_Rz_tmp_tmp = 3 * k + 2;
    c_x_data[b_Rz_tmp_tmp] = 0.0;
    Ry_tmp = 0.0;
    b_Ry_tmp = 0.0;
    Rz_tmp = 0.0;
    for (int i{0}; i < 3; i++) {
      b_Rz_tmp = c_x_data[i + 3 * k];
      Ry_tmp += R[3 * i] * b_Rz_tmp;
      b_Ry_tmp += R[3 * i + 1] * b_Rz_tmp;
      Rz_tmp += R[3 * i + 2] * b_Rz_tmp;
    }
    T_data[b_Rz_tmp_tmp] = Rz_tmp;
    T_data[Rz_tmp_tmp] = b_Ry_tmp;
    T_data[3 * k] = Ry_tmp;
  }
  for (int k{0}; k < 300; k++) {
    tmp_data[k] = 3501.4087480216976 * T_data[k];
  }
  for (int k{0}; k < 100; k++) {
    c_x_data[3 * k] = bx_data[k];
    c_x_data[3 * k + 1] = by_data[k];
    c_x_data[3 * k + 2] = bz_data[k];
  }
  coder::cross(tmp_data, c_x_data, p_data, theta_size);
  //  Compute torque
  for (int k{0}; k < 100; k++) {
    tmp_data[3 * k] = 0.0;
    Rz_tmp_tmp = 3 * k + 1;
    tmp_data[Rz_tmp_tmp] = 0.0;
    b_Rz_tmp_tmp = 3 * k + 2;
    tmp_data[b_Rz_tmp_tmp] = 1.0;
    Ry_tmp = 0.0;
    b_Ry_tmp = 0.0;
    Rz_tmp = 0.0;
    for (int i{0}; i < 3; i++) {
      b_Rz_tmp = tmp_data[i + 3 * k];
      Ry_tmp += R[3 * i] * b_Rz_tmp;
      b_Ry_tmp += R[3 * i + 1] * b_Rz_tmp;
      Rz_tmp += R[3 * i + 2] * b_Rz_tmp;
    }
    T_data[b_Rz_tmp_tmp] = Rz_tmp;
    T_data[Rz_tmp_tmp] = b_Ry_tmp;
    T_data[3 * k] = Ry_tmp;
  }
  for (int k{0}; k < 300; k++) {
    tmp_data[k] = 3501.4087480216976 * T_data[k];
  }
  for (int k{0}; k < 100; k++) {
    c_x_data[3 * k] = bx_data[k];
    c_x_data[3 * k + 1] = by_data[k];
    c_x_data[3 * k + 2] = bz_data[k];
  }
  coder::cross(tmp_data, c_x_data, T_data, theta_size);
  //  Setting up system matrices
  //  Mass and inertia properties of the magnet
  coder::diag(params.magnet.b_I, R);
  //  Computing the nonlinear function f(x,u)
  std::memset(&b[0], 0, 3U * sizeof(double));
  Ry_tmp = b[0];
  b_Ry_tmp = b[1];
  Rz_tmp = b[2];
  for (int k{0}; k < 3; k++) {
    b_Rz_tmp = x[k + 9];
    Ry_tmp += R[3 * k] * b_Rz_tmp;
    b_Ry_tmp += R[3 * k + 1] * b_Rz_tmp;
    Rz_tmp += R[3 * k + 2] * b_Rz_tmp;
  }
  b_theta_data[100] = 6.2831853071795862;
  e_p_data[100] = p_data[0];
  c_theta_data[100] = 6.2831853071795862;
  f_p_data[100] = p_data[1];
  d_theta_data[100] = 6.2831853071795862;
  g_p_data[100] = p_data[2];
  e_theta_data[100] = 6.2831853071795862;
  b_T_data[100] = T_data[0];
  f_theta_data[100] = 6.2831853071795862;
  c_T_data[100] = T_data[1];
  g_theta_data[100] = 6.2831853071795862;
  std::copy(&theta_data[0], &theta_data[100], &b_theta_data[0]);
  std::copy(&theta_data[0], &theta_data[100], &c_theta_data[0]);
  std::copy(&theta_data[0], &theta_data[100], &d_theta_data[0]);
  std::copy(&theta_data[0], &theta_data[100], &e_theta_data[0]);
  std::copy(&theta_data[0], &theta_data[100], &f_theta_data[0]);
  std::copy(&theta_data[0], &theta_data[100], &g_theta_data[0]);
  for (int k{0}; k < 100; k++) {
    e_p_data[k] = p_data[3 * k];
    Rz_tmp_tmp = 3 * k + 1;
    f_p_data[k] = p_data[Rz_tmp_tmp];
    b_Rz_tmp_tmp = 3 * k + 2;
    g_p_data[k] = p_data[b_Rz_tmp_tmp];
    b_T_data[k] = T_data[3 * k];
    c_T_data[k] = T_data[Rz_tmp_tmp];
    d_T_data[k] = T_data[b_Rz_tmp_tmp];
  }
  d_T_data[100] = T_data[2];
  dv[0] = 0.025 * coder::trapz(b_theta_data, e_p_data);
  dv[1] = 0.025 * coder::trapz(c_theta_data, f_p_data);
  dv[2] = 0.025 * coder::trapz(d_theta_data, g_p_data);
  dv[3] = 0.025 * coder::trapz(e_theta_data, b_T_data);
  dv[4] = 0.025 * coder::trapz(f_theta_data, c_T_data);
  dv[5] = 0.025 * coder::trapz(g_theta_data, d_T_data);
  dv1[0] = 0.0;
  dv1[1] = 0.0;
  dv1[2] = 0.0;
  dv1[3] = Rz_tmp * x[10] - b_Ry_tmp * x[11];
  dv1[4] = Ry_tmp * x[11] - Rz_tmp * x[9];
  dv1[5] = b_Ry_tmp * x[9] - Ry_tmp * x[10];
  for (int k{0}; k < 6; k++) {
    dv[k] -= dv1[k];
    dv2[6 * k] = dv3[3 * k];
    dv2[6 * k + 1] = dv3[3 * k + 1];
    dv2[6 * k + 2] = dv3[3 * k + 2];
  }
  for (int k{0}; k < 3; k++) {
    dv2[6 * k + 3] = 0.0;
    Rz_tmp_tmp = 6 * (k + 3);
    dv2[Rz_tmp_tmp + 3] = R[3 * k];
    dv2[6 * k + 4] = 0.0;
    dv2[Rz_tmp_tmp + 4] = R[3 * k + 1];
    dv2[6 * k + 5] = 0.0;
    dv2[Rz_tmp_tmp + 5] = R[3 * k + 2];
  }
  coder::mldivide(dv2, dv);
  for (int k{0}; k < 6; k++) {
    dv[k] -= dv4[k];
  }
  std::memset(&dx[0], 0, 12U * sizeof(double));
  for (int k{0}; k < 12; k++) {
    for (int i{0}; i < 12; i++) {
      dx[i] += static_cast<double>(b_a[i + 12 * k]) * x[k];
    }
  }
  std::memset(&a[0], 0, 12U * sizeof(double));
  for (int k{0}; k < 6; k++) {
    for (int i{0}; i < 12; i++) {
      a[i] += static_cast<double>(c_a[i + 12 * k]) * dv[k];
    }
  }
  for (int k{0}; k < 12; k++) {
    dx[k] += a[k];
  }
}

//
// Arguments    : const double x[12]
//                const double u[4]
//                double y[3]
// Return Type  : void
//
void maglevSystemMeasurements_fast(const double x[12], const double u[4],
                                   double y[3])
{
  static const double dv[3]{-0.0003, 0.0, 0.0};
  static const signed char iv[3]{1, 0, 0};
  double R[9];
  double b_Rz_tmp[9];
  double c_Rz_tmp[9];
  double b_x[3];
  double pRotated[3];
  double Rx_tmp;
  double Rz_tmp;
  double b_Rx_tmp;
  double bxBase;
  double bxTemp;
  double byBase;
  double byTemp;
  double bzBase;
  double bzTemp;
  int Rz_tmp_tmp;
  int b_Rz_tmp_tmp;
  if (!isInitialized_maglevModel) {
    maglevModel_initialize();
  }
  //  MAGLEVSYSTEMMEASUREMENTS implements the function h in the ODE
  //    dxdt = f(x,u);
  //       y = h(x,u);
  //  defining the sensor measurements of a magnetic levitation system. The
  //  system is fully defined by the params struct, and the magnetic model to
  //  be used for the measurements is defined by modelName, which can be either
  //  'fast', 'accurate' or 'filament'. The measurements computes the effect
  //  from all magnetic components of the system.
  //  Example:
  //    params; (from parameter file)
  //    modelName = 'fast';
  //    x = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
  //    u = [1,0,-1,0]';
  //    h = @(x,u) maglevSystemMeasuremets(x,u,params,modelName);
  //    y = h(x,u);
  //  See also MAGLEVSYSTEMDYNAMICS,
  //           COMPUTEFIELDTOTAL.
  //  MAGLEVSYSTEMMEASUREMENTS uses the implementation described in [1].
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  References:
  //  [1] Engmark, Hans Alvar, and Kiet Tuan Hoang.
  //      "Modeling and Control of a Magnetic Levitation Platform."
  //      IFAC-PapersOnLine 56.2 (2023): 7276-7281.
  //  COMPUTEFIELDTOTAL computes the magnetic field in cartesian coordinates
  //  produced by a base of permanent magnets and solenoids, defined by params,
  //  AND a levitating magnet defined by params and eta. The field is computed
  //  using the magnet/solenoid model defined by modelName.
  //  The function calculates the magnetic field components (bx, by, bz) at
  //  the specified points (x, y, z) in polar coordinates. u is the current in
  //  running through the solenoids (its size defined by the number of
  //  solenoids in params). modelName is either 'fast', 'accurate' or
  //  'fillament'.
  //  Example:
  //    x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
  //    u = [1,0,-1,0]'; eta = [0,0,0.05,0,0,0,0,0,0,0,0,0]';
  //    params; (from parameter file)
  //    modelName = 'fast';
  //    [bx,by,bz] = computeFieldTotal(x,y,z,eta,u,params,modelName);
  //  See also COMPUTEFIELDBASE,
  //           COMPUTEFORCEANDTORQUE.
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  Field from base
  //  COMPUTEFIELDBASE computes the magnetic field in cartesian coordinates
  //  produced by a base of permanent magnets and solenoids, defined by params,
  //  using the magnet/solenoid model defined by modelName.
  //  The function calculates the magnetic field components (bx, by, bz) at
  //  the specified points (x, y, z) in polar coordinates. u is the current in
  //  running through the solenoids (its size defined by the number of
  //  solenoids in params). modelName is either 'fast', 'accurate' or
  //  'fillament'.
  //  Example:
  //    x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
  //    u = [1,0,-1,0]';
  //    params; (from parameter file)
  //    modelName = 'fast';
  //    [bx,by,bz] = computeFieldBase(x,y,z,u,params,modelName);
  //  See also COMPUTEFIELDTOTAL,
  //           COMPUTEFORCEANDTORQUE.
  //  Author: Hans Alvar Engmark
  //  Date: 08.01.2024
  //  Initialize field array
  //  Field from permanent magnets
  //  Default is 'fast'
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.permanent.x[0], 0.0 - b_params.permanent.y[0], -0.0042,
      0.01, 7321.1273822271851, byTemp, bzTemp);
  bxBase = bxTemp;
  byBase = byTemp;
  bzBase = bzTemp;
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.permanent.x[1], 0.0 - b_params.permanent.y[1], -0.0042,
      0.01, 7321.1273822271851, byTemp, bzTemp);
  bxBase += bxTemp;
  byBase += byTemp;
  bzBase += bzTemp;
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.permanent.x[2], 0.0 - b_params.permanent.y[2], -0.0042,
      0.01, 7321.1273822271851, byTemp, bzTemp);
  bxBase += bxTemp;
  byBase += byTemp;
  bzBase += bzTemp;
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.permanent.x[3], 0.0 - b_params.permanent.y[3], -0.0042,
      0.01, 7321.1273822271851, byTemp, bzTemp);
  bxBase += bxTemp;
  byBase += byTemp;
  bzBase += bzTemp;
  //  Field from solenoids
  //  Default is 'fast'
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.solenoids.x[0], 0.0 - b_params.solenoids.y[0], -0.006,
      0.00786622775, u[0], byTemp, bzTemp);
  bxBase += bxTemp * 480.0;
  byBase += byTemp * 480.0;
  bzBase += bzTemp * 480.0;
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.solenoids.x[1], 0.0 - b_params.solenoids.y[1], -0.006,
      0.00786622775, u[1], byTemp, bzTemp);
  bxBase += bxTemp * 480.0;
  byBase += byTemp * 480.0;
  bzBase += bzTemp * 480.0;
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.solenoids.x[2], 0.0 - b_params.solenoids.y[2], -0.006,
      0.00786622775, u[2], byTemp, bzTemp);
  bxBase += bxTemp * 480.0;
  byBase += byTemp * 480.0;
  bzBase += bzTemp * 480.0;
  bxTemp = computeFieldCircularWireCartesian(
      -0.0003 - b_params.solenoids.x[3], 0.0 - b_params.solenoids.y[3], -0.006,
      0.00786622775, u[3], byTemp, bzTemp);
  bxBase += bxTemp * 480.0;
  byBase += byTemp * 480.0;
  bzBase += bzTemp * 480.0;
  //  Field from levitating magnet
  //  Relative position and rotation of points
  //  Euler rotation matrix
  Rx_tmp = std::sin(x[3]);
  b_Rx_tmp = std::cos(x[3]);
  byTemp = std::sin(x[4]);
  bzTemp = std::cos(x[4]);
  bxTemp = std::sin(x[5]);
  Rz_tmp = std::cos(x[5]);
  b_Rz_tmp[0] = Rz_tmp;
  b_Rz_tmp[3] = -bxTemp;
  b_Rz_tmp[6] = 0.0;
  b_Rz_tmp[1] = bxTemp;
  b_Rz_tmp[4] = Rz_tmp;
  b_Rz_tmp[7] = 0.0;
  R[0] = bzTemp;
  R[3] = 0.0;
  R[6] = byTemp;
  b_Rz_tmp[2] = 0.0;
  R[1] = 0.0;
  b_Rz_tmp[5] = 0.0;
  R[4] = 1.0;
  b_Rz_tmp[8] = 1.0;
  R[7] = 0.0;
  R[2] = -byTemp;
  R[5] = 0.0;
  R[8] = bzTemp;
  std::memset(&c_Rz_tmp[0], 0, 9U * sizeof(double));
  for (int i{0}; i < 3; i++) {
    byTemp = c_Rz_tmp[3 * i];
    Rz_tmp_tmp = 3 * i + 1;
    b_Rz_tmp_tmp = 3 * i + 2;
    for (int i1{0}; i1 < 3; i1++) {
      bzTemp = R[i1 + 3 * i];
      byTemp += b_Rz_tmp[3 * i1] * bzTemp;
      c_Rz_tmp[Rz_tmp_tmp] += b_Rz_tmp[3 * i1 + 1] * bzTemp;
      c_Rz_tmp[b_Rz_tmp_tmp] += b_Rz_tmp[3 * i1 + 2] * bzTemp;
    }
    c_Rz_tmp[3 * i] = byTemp;
  }
  b_Rz_tmp[1] = 0.0;
  b_Rz_tmp[4] = b_Rx_tmp;
  b_Rz_tmp[7] = -Rx_tmp;
  b_Rz_tmp[2] = 0.0;
  b_Rz_tmp[5] = Rx_tmp;
  b_Rz_tmp[8] = b_Rx_tmp;
  std::memset(&R[0], 0, 9U * sizeof(double));
  for (int i{0}; i < 3; i++) {
    b_Rz_tmp[3 * i] = iv[i];
    byTemp = R[3 * i];
    Rz_tmp_tmp = 3 * i + 1;
    b_Rz_tmp_tmp = 3 * i + 2;
    for (int i1{0}; i1 < 3; i1++) {
      bzTemp = b_Rz_tmp[i1 + 3 * i];
      byTemp += c_Rz_tmp[3 * i1] * bzTemp;
      R[Rz_tmp_tmp] += c_Rz_tmp[3 * i1 + 1] * bzTemp;
      R[b_Rz_tmp_tmp] += c_Rz_tmp[3 * i1 + 2] * bzTemp;
    }
    R[3 * i] = byTemp;
  }
  coder::inv(R, b_Rz_tmp);
  std::memset(&pRotated[0], 0, 3U * sizeof(double));
  bzTemp = pRotated[0];
  bxTemp = pRotated[1];
  Rz_tmp = pRotated[2];
  for (int i{0}; i < 3; i++) {
    byTemp = x[i] - dv[i];
    bzTemp += b_Rz_tmp[3 * i] * byTemp;
    bxTemp += b_Rz_tmp[3 * i + 1] * byTemp;
    Rz_tmp += b_Rz_tmp[3 * i + 2] * byTemp;
  }
  //  Compute Field
  //  Default is 'fast'
  byTemp = computeFieldCircularWireCartesian(
      bzTemp, bxTemp, Rz_tmp, 0.025, 1750.7043740108488, Rx_tmp, b_Rx_tmp);
  b_x[0] = byTemp;
  b_x[1] = Rx_tmp;
  b_x[2] = b_Rx_tmp;
  std::memset(&pRotated[0], 0, 3U * sizeof(double));
  byTemp = pRotated[0];
  bzTemp = pRotated[1];
  bxTemp = pRotated[2];
  for (int i{0}; i < 3; i++) {
    Rz_tmp = b_x[i];
    byTemp += R[3 * i] * Rz_tmp;
    bzTemp += R[3 * i + 1] * Rz_tmp;
    bxTemp += R[3 * i + 2] * Rz_tmp;
  }
  //  Total field
  y[0] = bxBase + byTemp;
  y[1] = byBase + bzTemp;
  y[2] = bzBase + bxTemp;
}

//
// File trailer for maglevModel.cpp
//
// [EOF]
//
