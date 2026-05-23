/**
 * observer_capi.cpp
 *
 * C-linkage wrapper around the C++ KalmanFilter library.
 *
 * Build as a shared library, e.g.:
 *
 *   Linux / macOS:
 *     g++ -std=c++17 -shared -fPIC -O2 \
 *         observer_capi.cpp -o libobserver.so \
 *         -I<path/to/kalman/include> -L<path/to/kalman/lib> -lkalman
 *
 *   Windows (MSVC):
 *     cl /std:c++17 /LD observer_capi.cpp \
 *        /I<include> /link /LIBPATH:<lib> kalman.lib /OUT:observer.dll
 *
 *   Windows (MinGW / g++):
 *     g++ -std=c++17 -shared -O2 \
 *         observer_capi.cpp -o observer.dll \
 *         -I<include> -L<lib> -lkalman
 */

#include "observer_capi.h"
#include "observer.h"

#include <unordered_map>
#include <mutex>
#include <atomic>
#include <stdexcept>
#include <chrono>

/* -----------------------------------------------------------------------
 * Internal handle registry
 * A simple thread-safe map: int handle -> (filterPtr, nx, nu, ny)
 * ----------------------------------------------------------------------- */
namespace {

struct Entry {
    filterPtr   filter;
    int         nx;
    int         nu;
    int         ny;
    double last_runtime_us;
};

std::unordered_map<int, Entry> g_registry;
std::mutex                     g_mutex;
std::atomic<int>               g_nextHandle{0};

Entry* find(int handle) {
    auto it = g_registry.find(handle);
    return (it != g_registry.end()) ? &it->second : nullptr;
}

} // anonymous namespace


/* -----------------------------------------------------------------------
 * Public C API
 * ----------------------------------------------------------------------- */

extern "C" {

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
                             int    cubature)
{
    try {
        filterPtr fp = initObserver(
            filterVariant, dt, static_cast<size_t>(nx),
            x0, R, Q, P0,
            static_cast<bool>(useSRformulation),
            RK4Iterations,
            static_cast<bool>(updateJacobians),
            static_cast<bool>(updateQ),
            static_cast<bool>(cubature)
        );

        if (!fp) return OBSERVER_INVALID_HANDLE;

        /* We need nu/ny at run-time for validation.
         * Retrieve them from the filter object if accessible, or
         * store the values from the header constants as fall-backs. */
        int handle = g_nextHandle.fetch_add(1);
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            g_registry[handle] = Entry{
                std::move(fp),
                nx,
                NUMBER_INPUTS,        /* compile-time constants from header */
                NUMBER_MEASUREMENTS
            };
        }
        return handle;
    }
    catch (...) {
        return OBSERVER_INVALID_HANDLE;
    }
}


int observer_run(ObserverHandle handle,
                 const double*  input,  int nu,
                 const double*  meas,   int ny,
                 double*        stateEstimates, int nx_out)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    Entry* e = find(handle);
    if (!e) return -1;

    /* Optional dimension guard */
    if (nu != e->nu || ny != e->ny || nx_out != e->nx) return -2;

    auto t0 = std::chrono::steady_clock::now();
    runObserver(input, meas, stateEstimates, *e->filter);
    auto t1 = std::chrono::steady_clock::now();
    e->last_runtime_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    return 0;
}


void observer_destroy(ObserverHandle handle)
{
    if (handle == OBSERVER_INVALID_HANDLE) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    g_registry.erase(handle);   /* unique_ptr destructor fires here */
}


int observer_get_nx(ObserverHandle handle)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    Entry* e = find(handle);
    return e ? e->nx : -1;
}

int observer_get_nu(ObserverHandle handle)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    Entry* e = find(handle);
    return e ? e->nu : -1;
}

int observer_get_ny(ObserverHandle handle)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    Entry* e = find(handle);
    return e ? e->ny : -1;
}

double observer_get_last_runtime_us(ObserverHandle handle)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    Entry* e = find(handle);
    return e ? e->last_runtime_us : -1.0;
}

// Helper macro to reduce boilerplate — expands to a full getter function.
// mat_expr is any expression that yields an arma::mat given a KalmanFilter&.
#define DEFINE_MAT_GETTER(fn_name, mat_expr)                                  \
int fn_name(ObserverHandle handle, double* out,                               \
            int* rows_out, int* cols_out)                                     \
{                                                                             \
    std::lock_guard<std::mutex> lk(g_mutex);                                  \
    Entry* e = find(handle);                                                  \
    if (!e) return -1;                                                        \
    arma::mat result = (e->filter->mat_expr);                                 \
    *rows_out = static_cast<int>(result.n_rows);                              \
    *cols_out = static_cast<int>(result.n_cols);                              \
    std::memcpy(out, result.memptr(), result.n_elem * sizeof(double));        \
    return 0;                                                                 \
}

DEFINE_MAT_GETTER(observer_get_innovation_cov, getInnovationCovariance())
DEFINE_MAT_GETTER(observer_get_innovation, getInnovation())
DEFINE_MAT_GETTER(observer_get_meas_pred, getMeasPred())
DEFINE_MAT_GETTER(observer_get_state_cov, getCovariance())
DEFINE_MAT_GETTER(observer_get_state, getState())
DEFINE_MAT_GETTER(observer_get_kalman_gain, getKalmanGain())

} // extern "C"
