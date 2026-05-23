/**
 * observer_capi.h
 *
 * Pure-C interface around the C++ KalmanFilter library.
 * Compile the accompanying .cpp into a shared library; MATLAB loads it
 * with loadlibrary() using this header.
 *
 * Handle lifecycle:
 *   handle = observer_init(...)   -- allocates filter, returns integer handle
 *   observer_run(handle, ...)     -- runs one filter step
 *   observer_destroy(handle)      -- frees memory
 */

#ifndef OBSERVER_CAPI_H
#define OBSERVER_CAPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque integer handle; MATLAB stores this as int32. */
typedef int ObserverHandle;
#define OBSERVER_INVALID_HANDLE (-1)

/**
 * Initialise a new filter instance.
 *
 * @param filterVariant     Filter variant selector (passed to initObserver)
 * @param dt                Sample time [s]
 * @param nx                Length of x0
 * @param x0                Initial state and linearization point
 * @param R                 Measurement noise covariance
 * @param Q                 Process noise covariance
 * @param P0                Initial covariance
 * @param useSRformulation  0 = false, 1 = true
 * @param RK4Iterations     Number of RK4 sub-steps (0 = default)
 * @param updateJacobians   0 = false, 1 = true
 * @param updateQ           0 = false, 1 = true
 * @param cubature          0 = false, 1 = true
 *
 * @return  Integer handle (>= 0) on success, OBSERVER_INVALID_HANDLE on failure.
 */
ObserverHandle observer_init(int    filterVariant,
                             double dt,
                             int    nx,
			     double *x0, /* nx x nx */
			     double *R,  /* 3 x 3 */
			     double *Q,  /* nx x nx */
			     double *P0, /* nx x nx */
                             int    useSRformulation,
                             int    RK4Iterations,
                             int    updateJacobians,
                             int    updateQ,
                             int    cubature);

/**
 * Run one filter step.
 *
 * @param handle          Handle returned by observer_init
 * @param input           Input vector  (length nu)
 * @param nu              Length of input
 * @param meas            Measurement vector (length ny)
 * @param nz              Length of meas
 * @param stateEstimates  Output buffer to fill  (length nx)
 * @param nx              Length of stateEstimates
 *
 * @return  0 on success, -1 on bad handle, -2 on dimension mismatch.
 */
int observer_run(ObserverHandle handle,
                 const double*  input,  int nu,
                 const double*  meas,   int nz,
                 double*        stateEstimates, int nx);

/**
 * Free the filter instance associated with handle.
 * Safe to call with OBSERVER_INVALID_HANDLE (no-op).
 */
void observer_destroy(ObserverHandle handle);

/**
 * Query the number of states / inputs / measurements the stored filter
 * was constructed with.  Useful for pre-allocating MATLAB buffers.
 * Returns -1 for an unknown handle.
 */
int observer_get_nx(ObserverHandle handle);
int observer_get_nu(ObserverHandle handle);
int observer_get_ny(ObserverHandle handle);

/**
 * Generic matrix getter pattern.
 * out      – caller-allocated buffer, size rows_out * cols_out * sizeof(double)
 *            safe upper bound: nx*nx for square matrices, ny*ny for S
 * rows_out / cols_out – filled by the function
 * Returns 0 on success, -1 bad handle.
 */
int observer_get_innovation_cov (ObserverHandle handle, double* out, int* rows_out, int* cols_out);
int observer_get_innovation     (ObserverHandle handle, double* out, int* rows_out, int* cols_out);
int observer_get_meas_pred      (ObserverHandle handle, double* out, int* rows_out, int* cols_out);
int observer_get_state_cov      (ObserverHandle handle, double* out, int* rows_out, int* cols_out);
int observer_get_state          (ObserverHandle handle, double* out, int* rows_out, int* cols_out);
int observer_get_kalman_gain    (ObserverHandle handle, double* out, int* rows_out, int* cols_out);
double observer_get_last_runtime_us(ObserverHandle handle);


#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_CAPI_H */
