
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
/*
#define dsyevd_ dsyevd
#define dsyev_ dsyev
#define dgemv_ dgemv
#define dgemm_ dgemm
#define ddot_ ddot
#define dsyrk_ dsyrk
#define dpotrf_ dpotrf
#define dgeqrf__ dgeqrf
#define dorgqr__ dorgqr
#define dpbtrf__ dpbtrf
#define dqeqrf__ dqeqrf
#define dgbsvx__ dgbsvx
#define dgbsvy__ dgbsvy
#define dgbcon__ dgbcon
#define dlansy__ dlansy
#define dpotrs__ dpotrs
#define dpocon__ dpocon
#define dsytrf__ dsytrf
#define dsycon__ dsycon
#define dposv__ dposv
#define dgels__ dgels
#define dgecon__ dgecon
#define dgbtrf__ dgbtrf
#define dgbtrs__ dgbtrs
#define dlange__ dlange
#define ilaenv__ ilaenv
#define dgelsd__ dgelsd
#define dtrtrs__ dtrtrs
#define dgesv__ dgesv
#define dtrcon__ dtrcon
#define dposvx__ dposvx
#define dgesvx__ dgesvx
#define dgetrf__ dgetrf
#define dgbsv__ dgbsv
#define dsytrs__ dsytrs
#define dgtsv__ dgtsv
#define dgvsv__ dgvsv
#define dgetrs__ dgetrs
*/

//#define ARMA_BLAS_LONG_LONG
#define ARMA_USE_BLAS
#define ARMA_USE_LAPACK

#include <armadillo>

#include "/home/miku/MSc/Maggy2026V/observer/PCobserver/observer.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 4
#define u_1_width 3
#define y_width 6

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
extern "C" void observerBlock_Start_wrapper(void **pW);

void observerBlock_Start_wrapper(void **pW)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */
int filterVariant = 2;
bool useSRformulation = 0;
int RK4Iterations = 0;
bool updateJacobians = 1;
bool updateQ = 0;
bool cubature = 0;

double dt = 0.01;

double zEq = 0.030119178665731;

arma::vec xLp(6, arma::fill::zeros);
xLp(2) = zEq;

std::unique_ptr<KalmanFilter> observer = initObserver(filterVariant, dt, xLp, useSRformulation, RK4Iterations, updateJacobians, updateQ, cubature);

// Release ownership to PWork 
pW[0] = observer.release();
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern "C" void observerBlock_Outputs_wrapper(const real_T *input,
			const real_T *meas,
			real_T *x_est,
			void **pW);

void observerBlock_Outputs_wrapper(const real_T *input,
			const real_T *meas,
			real_T *x_est,
			void **pW)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
KalmanFilter *obs = static_cast<KalmanFilter*>(pW[0]);
runObserver(input, meas, x_est, *obs);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
extern "C" void observerBlock_Terminate_wrapper(void **pW);

void observerBlock_Terminate_wrapper(void **pW)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Terminate code goes here.
 */
delete static_cast<KalmanFilter*>(pW[0]);
pW[0] = nullptr;
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

