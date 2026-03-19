// observer_mex.cpp
#include "/usr/local/MATLAB/R2025b/extern/include/mex.h"
#include "observer.h"
#include <string.h>
#include <vector>

// Persistent data structure to maintain state between calls
class ObserverData {
public:
    bool initialized;
    size_t currentFilter;
    
    ObserverData() : initialized(false), currentFilter(1) {}  // Default to EKF
};

// Cleanup function called when MEX is cleared
void cleanup(void) {
    // Any necessary cleanup (none currently needed)
}

// Helper function to check number of inputs/outputs
void checkArguments(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Minimum arguments check
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("observer:usage", 
            "Usage:\n"
            "  observer_mex('init', [filterType]) - Initialize with optional filter type (0=KF,1=EKF,2=UKF)\n"
            "  observer_mex(inputs, measurements, [filterType]) - Run observer\n"
            "  observer_mex('getNIS') - Get current NIS value\n"
            "  observer_mex('setFilter', filterType) - Change filter type\n"
            "  observer_mex('getFilter') - Get current filter type");
    }
}

// Parse filter type from MATLAB input
size_t parseFilterType(const mxArray* input, const char* argName) {
    if (!mxIsDouble(input) || mxGetNumberOfElements(input) != 1) {
        mexErrMsgIdAndTxt("observer:invalidFilter", 
            "%s must be a scalar double (0=KF, 1=EKF, 2=UKF)", argName);
    }
    
    double val = mxGetScalar(input);
    if (val < 0 || val > 2 || val != floor(val)) {
        mexErrMsgIdAndTxt("observer:invalidFilter", 
            "Filter type must be integer 0, 1, or 2");
    }
    
    return static_cast<size_t>(val);
}

// MEX gateway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    // Get persistent data
    static ObserverData* data = nullptr;
    if (!data) {
        data = new ObserverData();
        mexMakeMemoryPersistent(data);
        mexAtExit(cleanup);
    }
    
    checkArguments(nlhs, plhs, nrhs, prhs);
    
    // Check if first argument is a string command
    if (mxIsChar(prhs[0])) {
        char cmd[64];
        mxGetString(prhs[0], cmd, sizeof(cmd));
        
        // Handle 'init' command
        if (strcmp(cmd, "init") == 0) {
            size_t filterType = 1;  // Default to EKF
            
            // Check if filter type was provided
            if (nrhs >= 2) {
                filterType = parseFilterType(prhs[1], "Filter type");
            }
            
            // Initialize observer with specified filter
            initObserver(filterType);
            data->initialized = true;
            data->currentFilter = filterType;
            
            mexPrintf("Observer initialized with filter type: %s\n", 
                filterType == 0 ? "KF" : (filterType == 1 ? "EKF" : "UKF"));
            
            return;
        }
        
        // Handle 'getNIS' command
        else if (strcmp(cmd, "getNIS") == 0) {
            if (!data->initialized) {
                mexErrMsgIdAndTxt("observer:notInit", 
                    "Observer not initialized. Call observer_mex('init') first");
            }
            
            plhs[0] = mxCreateDoubleScalar(NIS);
            return;
        }
        
        // Handle 'setFilter' command
        else if (strcmp(cmd, "setFilter") == 0) {
            if (nrhs < 2) {
                mexErrMsgIdAndTxt("observer:invalidInput", 
                    "setFilter requires filter type argument");
            }
            
            size_t filterType = parseFilterType(prhs[1], "Filter type");
            
            // Re-initialize with new filter type
            initObserver(filterType);
            data->currentFilter = filterType;
            
            mexPrintf("Filter changed to: %s\n", 
                filterType == 0 ? "KF" : (filterType == 1 ? "EKF" : "UKF"));
            
            return;
        }
        
        // Handle 'getFilter' command
        else if (strcmp(cmd, "getFilter") == 0) {
            plhs[0] = mxCreateDoubleScalar(static_cast<double>(data->currentFilter));
            return;
        }
        
        else {
            mexErrMsgIdAndTxt("observer:invalidCmd", 
                "Unknown command: %s", cmd);
        }
    }
    
    // If not a command, it should be the run observer case
    else {
        // Check if initialized
        if (!data->initialized) {
            mexErrMsgIdAndTxt("observer:notInit", 
                "Observer not initialized. Call observer_mex('init') first");
        }
        
        // Check number of inputs for run case
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("observer:invalidInput", 
                "Need at least inputs and measurements for run");
        }
        
        // Parse filter type if provided (optional third argument)
        size_t filterType = data->currentFilter;  // Use current filter by default
        if (nrhs >= 3) {
            filterType = parseFilterType(prhs[2], "Filter type");
            if (filterType != data->currentFilter) {
                mexWarnMsgIdAndTxt("observer:filterMismatch", 
                    "Using different filter type than current. Current: %d, Requested: %d\n"
                    "Use observer_mex('setFilter', type) to permanently change filter.",
                    data->currentFilter, filterType);
            }
        }
        
        // Extract inputs
        if (mxGetNumberOfElements(prhs[0]) != NUMBER_INPUTS) {
            mexErrMsgIdAndTxt("observer:inputSize", 
                "Input must have %d elements. Got %d.", 
                NUMBER_INPUTS, mxGetNumberOfElements(prhs[0]));
        }
        
        double* inputPtr = mxGetPr(prhs[0]);
        double input[NUMBER_INPUTS];
        for (int i = 0; i < NUMBER_INPUTS; i++) {
            input[i] = inputPtr[i];
        }
        
        // Extract measurements
        // Handle both row and column vectors
        if (mxGetNumberOfElements(prhs[1]) != NUMBER_MEASUREMENTS) {
            mexErrMsgIdAndTxt("observer:measSize", 
                "Measurements must have %d elements. Got %d.", 
                NUMBER_MEASUREMENTS, mxGetNumberOfElements(prhs[1]));
        }
        
        double* measPtr = mxGetPr(prhs[1]);
        double meas[1][NUMBER_MEASUREMENTS];  // Single row as required by runObserver
        for (int i = 0; i < NUMBER_MEASUREMENTS; i++) {
            meas[0][i] = measPtr[i];
        }
        
        // Output array
        double stateEstimates[NUMBER_STATES];
        
        // Run observer with specified filter
        runObserver(input, meas, stateEstimates, filterType);
        
        // Create output for state estimates
        plhs[0] = mxCreateDoubleMatrix(NUMBER_STATES, 1, mxREAL);
        double* outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < NUMBER_STATES; i++) {
            outPtr[i] = stateEstimates[i];
        }
        
        // Optional second output for NIS
        if (nlhs >= 2) {
            plhs[1] = mxCreateDoubleScalar(NIS);
        }
        
        // Optional third output for filter type used
        if (nlhs >= 3) {
            plhs[2] = mxCreateDoubleScalar(static_cast<double>(filterType));
        }
    }
}
