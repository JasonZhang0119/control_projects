#include "mex.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "estimator/estimator_gradient.hpp"

#ifndef GSE_N_Y
#define GSE_N_Y 1
#endif

#ifndef GSE_N_THETA
#define GSE_N_THETA 2
#endif

using EstimatorType = GradientEstimator_Standard<double, GSE_N_Y, GSE_N_THETA>;
using EstimatorMap = std::unordered_map<std::uint64_t, std::unique_ptr<EstimatorType>>;

static EstimatorMap estimators;
static std::uint64_t next_handle = 1;

[[noreturn]] static void RaiseError(const char* message)
{
    mexErrMsgIdAndTxt("control_lib:gradient_standard_estimator:error", "%s", message);
}

static double ReadScalar(const mxArray* input, const char* name)
{
    if (!mxIsDouble(input) || mxIsComplex(input) || mxGetNumberOfElements(input) != 1)
    {
        mexErrMsgIdAndTxt(
            "control_lib:gradient_standard_estimator:invalidScalar",
            "%s must be a real double scalar.",
            name);
    }

    return mxGetScalar(input);
}

static std::uint64_t ReadHandle(const mxArray* input)
{
    if (!mxIsUint64(input) || mxIsComplex(input) || mxGetNumberOfElements(input) != 1)
    {
        RaiseError("Estimator handle must be a uint64 scalar.");
    }

    return *static_cast<const std::uint64_t*>(mxGetData(input));
}

static EstimatorType& FindEstimator(const mxArray* input)
{
    const std::uint64_t handle = ReadHandle(input);
    const auto iterator = estimators.find(handle);

    if (iterator == estimators.end())
    {
        RaiseError("Invalid or destroyed estimator handle.");
    }

    return *(iterator->second);
}

static EstimatorType::RegressorMatrix ReadRegressor(const mxArray* input)
{
    constexpr int expected_elements = GSE_N_THETA * GSE_N_Y;

    if (!mxIsDouble(input) || mxIsComplex(input) ||
        mxGetNumberOfElements(input) != expected_elements)
    {
        RaiseError("Regressor must be a real double matrix with GSE_N_THETA * GSE_N_Y elements.");
    }

    EstimatorType::RegressorMatrix regressor;
    const double* data = mxGetDoubles(input);

    for (int column = 0; column < GSE_N_Y; ++column)
    {
        for (int row = 0; row < GSE_N_THETA; ++row)
        {
            const int index = row + column * GSE_N_THETA;
            regressor(row, column) = data[index];
        }
    }

    return regressor;
}

static EstimatorType::MeasurementVector ReadMeasurement(const mxArray* input)
{
    if (!mxIsDouble(input) || mxIsComplex(input) ||
        mxGetNumberOfElements(input) != GSE_N_Y)
    {
        RaiseError("Measurement must be a real double vector with GSE_N_Y elements.");
    }

    EstimatorType::MeasurementVector measurement;
    const double* data = mxGetDoubles(input);

    for (int index = 0; index < GSE_N_Y; ++index)
    {
        measurement(index) = data[index];
    }

    return measurement;
}

static mxArray* CreateParameterOutput(const EstimatorType& estimator)
{
    mxArray* output = mxCreateDoubleMatrix(GSE_N_THETA, 1, mxREAL);
    double* data = mxGetDoubles(output);

    for (int index = 0; index < GSE_N_THETA; ++index)
    {
        data[index] = estimator.GetParameter()(index);
    }

    return output;
}

static void RequireInputCount(int nrhs, int expected_count, const char* command)
{
    if (nrhs != expected_count)
    {
        mexErrMsgIdAndTxt(
            "control_lib:gradient_standard_estimator:invalidInputCount",
            "%s received an incorrect number of inputs.",
            command);
    }
}

static void RequireOutputCount(int nlhs, int expected_count, const char* command)
{
    if (nlhs != expected_count)
    {
        mexErrMsgIdAndTxt(
            "control_lib:gradient_standard_estimator:invalidOutputCount",
            "%s received an incorrect number of output arguments.",
            command);
    }
}

static void ConfigureEstimator(EstimatorType& estimator, const mxArray* prhs[])
{
    estimator.SetRegressor(ReadRegressor(prhs[2]));
    estimator.SetMeasurement(ReadMeasurement(prhs[3]));
    estimator.SetTs(static_cast<float>(ReadScalar(prhs[4], "Ts")));
    estimator.SetGamma(static_cast<float>(ReadScalar(prhs[5], "gamma")));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0]))
    {
        RaiseError("The first input must be a command string.");
    }

    char command_buffer[64];
    if (mxGetString(prhs[0], command_buffer, sizeof(command_buffer)) != 0)
    {
        RaiseError("Command string is too long.");
    }

    const std::string command(command_buffer);

    if (command == "create")
    {
        RequireInputCount(nrhs, 1, "create");
        RequireOutputCount(nlhs, 1, "create");

        const std::uint64_t handle = next_handle++;
        estimators.emplace(handle, std::make_unique<EstimatorType>());
        mexLock();

        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *static_cast<std::uint64_t*>(mxGetData(plhs[0])) = handle;
    }
    else if (command == "destroy")
    {
        RequireInputCount(nrhs, 2, "destroy");
        RequireOutputCount(nlhs, 0, "destroy");

        const std::uint64_t handle = ReadHandle(prhs[1]);
        const auto iterator = estimators.find(handle);

        if (iterator == estimators.end())
        {
            RaiseError("Invalid or already destroyed estimator handle.");
        }

        estimators.erase(iterator);
        mexUnlock();
    }
    else if (command == "reset")
    {
        RequireInputCount(nrhs, 2, "reset");
        RequireOutputCount(nlhs, 0, "reset");
        FindEstimator(prhs[1]).Reset();
    }
    else if (command == "step")
    {
        RequireInputCount(nrhs, 6, "step");
        RequireOutputCount(nlhs, 1, "step");

        EstimatorType& estimator = FindEstimator(prhs[1]);
        ConfigureEstimator(estimator, prhs);
        estimator.Compute();
        plhs[0] = CreateParameterOutput(estimator);
    }
    else if (command == "get_state")
    {
        RequireInputCount(nrhs, 2, "get_state");
        RequireOutputCount(nlhs, 1, "get_state");
        plhs[0] = CreateParameterOutput(FindEstimator(prhs[1]));
    }
    else
    {
        RaiseError("Unknown command.");
    }
}
