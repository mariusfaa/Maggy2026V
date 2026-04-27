#include "/usr/local/MATLAB/R2025b/extern/include/mex.h"
#include "observer_registry.h"

// [xEst] = mexRunObserver(handle, input, meas)
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[])
{
    if (nrhs != 3)
        mexErrMsgIdAndTxt("obs:args", "Expected 3 inputs: handle, input, meas");

    uint64_t handle = *reinterpret_cast<uint64_t*>(mxGetData(prhs[0]));
    KalmanFilter& obs = ObserverRegistry::instance().get(handle);

    const double* u    = mxGetPr(prhs[1]);
    const double* meas = mxGetPr(prhs[2]);

    // Output buffer
    plhs[0] = mxCreateDoubleMatrix(NUMBER_STATES_REDUCED, 1, mxREAL);
    double* xEst = mxGetPr(plhs[0]);

    runObserver(u, meas, xEst, obs);
}
