#define S_FUNCTION_NAME sfun_recursive_least_square
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <new>

#include "parameter_estimator.hpp"

#ifndef RLS_N_THETA
#define RLS_N_THETA 2
#endif

typedef LeastSquareEstimator_Recursive<real_T, RLS_N_THETA> EstimatorType;

enum InputPortIndex
{
    INPUT_REGRESSOR = 0,
    INPUT_MEASUREMENT,
    INPUT_FORGETTING_FACTOR,
    INPUT_PORT_COUNT
};

enum OutputPortIndex
{
    OUTPUT_PARAMETER = 0,
    OUTPUT_GAIN,
    OUTPUT_INNOVATION,
    OUTPUT_COVARIANCE,
    OUTPUT_PORT_COUNT
};

enum ParameterIndex
{
    PARAMETER_INITIAL_COVARIANCE = 0,
    PARAMETER_COUNT
};

#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct* S)
{
    const mxArray* initial_covariance =
        ssGetSFcnParam(S, PARAMETER_INITIAL_COVARIANCE);

    if (!mxIsDouble(initial_covariance) ||
        mxIsComplex(initial_covariance) ||
        (mxGetNumberOfElements(initial_covariance) != 1) ||
        (mxGetScalar(initial_covariance) <= 0.0))
    {
        ssSetErrorStatus(S, "Initial covariance must be a positive real double scalar.");
    }
}

static void mdlInitializeSizes(SimStruct* S)
{
    ssSetNumSFcnParams(S, PARAMETER_COUNT);

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }

    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != nullptr)
    {
        return;
    }

    ssSetSFcnParamTunable(S, PARAMETER_INITIAL_COVARIANCE, 0);

    if (!ssSetNumInputPorts(S, INPUT_PORT_COUNT))
    {
        return;
    }

    ssSetInputPortWidth(S, INPUT_REGRESSOR, RLS_N_THETA);
    ssSetInputPortDataType(S, INPUT_REGRESSOR, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_REGRESSOR, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_REGRESSOR, 1);

    ssSetInputPortWidth(S, INPUT_MEASUREMENT, 1);
    ssSetInputPortDataType(S, INPUT_MEASUREMENT, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_MEASUREMENT, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_MEASUREMENT, 1);

    ssSetInputPortWidth(S, INPUT_FORGETTING_FACTOR, 1);
    ssSetInputPortDataType(S, INPUT_FORGETTING_FACTOR, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_FORGETTING_FACTOR, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_FORGETTING_FACTOR, 1);

    if (!ssSetNumOutputPorts(S, OUTPUT_PORT_COUNT))
    {
        return;
    }

    ssSetOutputPortWidth(S, OUTPUT_PARAMETER, RLS_N_THETA);
    ssSetOutputPortDataType(S, OUTPUT_PARAMETER, SS_DOUBLE);

    ssSetOutputPortWidth(S, OUTPUT_GAIN, RLS_N_THETA);
    ssSetOutputPortDataType(S, OUTPUT_GAIN, SS_DOUBLE);

    ssSetOutputPortWidth(S, OUTPUT_INNOVATION, 1);
    ssSetOutputPortDataType(S, OUTPUT_INNOVATION, SS_DOUBLE);

    ssSetOutputPortWidth(
        S, OUTPUT_COVARIANCE, RLS_N_THETA * RLS_N_THETA);
    ssSetOutputPortDataType(S, OUTPUT_COVARIANCE, SS_DOUBLE);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
    ssSetOptions(S, SS_OPTION_WORKS_WITH_CODE_REUSE);
}

static void mdlInitializeSampleTimes(SimStruct* S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct* S)
{
    EstimatorType* estimator = new (std::nothrow) EstimatorType();

    if (estimator == nullptr)
    {
        ssSetErrorStatus(S, "Failed to allocate the Recursive Least Square estimator.");
        return;
    }

    const real_T initial_covariance =
        mxGetScalar(ssGetSFcnParam(S, PARAMETER_INITIAL_COVARIANCE));
    const EstimatorType::CovarianceMatrix covariance =
        EstimatorType::CovarianceMatrix::Identity() * initial_covariance;

    estimator->Reset();
    estimator->SetCovariance(covariance);
    ssSetPWorkValue(S, 0, estimator);
}

static bool ConfigureEstimatorFromInputs(
    SimStruct* S,
    EstimatorType& estimator)
{
    const real_T* regressor_input = static_cast<const real_T*>(
        ssGetInputPortSignal(S, INPUT_REGRESSOR));
    const real_T* measurement_input = static_cast<const real_T*>(
        ssGetInputPortSignal(S, INPUT_MEASUREMENT));
    const real_T* forgetting_factor_input = static_cast<const real_T*>(
        ssGetInputPortSignal(S, INPUT_FORGETTING_FACTOR));

    EstimatorType::RegressorVector regressor;
    for (int index = 0; index < RLS_N_THETA; ++index)
    {
        regressor(index) = regressor_input[index];
    }

    if (!estimator.SetForgettingFactor(forgetting_factor_input[0]))
    {
        ssSetErrorStatus(S, "Forgetting factor must be in the interval (0, 1].");
        return false;
    }

    estimator.SetRegressor(regressor);
    estimator.SetMeasurement(measurement_input[0]);
    return true;
}

static void CopyEstimatorOutputs(
    SimStruct* S,
    const EstimatorType& estimator)
{
    real_T* parameter_output =
        static_cast<real_T*>(ssGetOutputPortSignal(S, OUTPUT_PARAMETER));
    real_T* gain_output =
        static_cast<real_T*>(ssGetOutputPortSignal(S, OUTPUT_GAIN));
    real_T* innovation_output =
        static_cast<real_T*>(ssGetOutputPortSignal(S, OUTPUT_INNOVATION));
    real_T* covariance_output =
        static_cast<real_T*>(ssGetOutputPortSignal(S, OUTPUT_COVARIANCE));

    for (int index = 0; index < RLS_N_THETA; ++index)
    {
        parameter_output[index] = estimator.GetParameter()(index);
        gain_output[index] = estimator.GetGain()(index);
    }

    innovation_output[0] = estimator.GetInnovation();

    for (int column = 0; column < RLS_N_THETA; ++column)
    {
        for (int row = 0; row < RLS_N_THETA; ++row)
        {
            const int output_index = row + column * RLS_N_THETA;
            covariance_output[output_index] = estimator.GetCovariance()(row, column);
        }
    }
}

static void mdlOutputs(SimStruct* S, int_T tid)
{
    (void)tid;

    EstimatorType* estimator =
        static_cast<EstimatorType*>(ssGetPWorkValue(S, 0));

    if (estimator == nullptr)
    {
        ssSetErrorStatus(S, "Recursive Least Square estimator is not initialized.");
        return;
    }

    EstimatorType output_estimator(*estimator);

    if (!ConfigureEstimatorFromInputs(S, output_estimator))
    {
        return;
    }

    output_estimator.Compute();
    CopyEstimatorOutputs(S, output_estimator);
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct* S, int_T tid)
{
    (void)tid;

    EstimatorType* estimator =
        static_cast<EstimatorType*>(ssGetPWorkValue(S, 0));

    if (estimator == nullptr)
    {
        ssSetErrorStatus(S, "Recursive Least Square estimator is not initialized.");
        return;
    }

    if (!ConfigureEstimatorFromInputs(S, *estimator))
    {
        return;
    }

    estimator->Compute();
}

static void mdlTerminate(SimStruct* S)
{
    EstimatorType* estimator =
        static_cast<EstimatorType*>(ssGetPWorkValue(S, 0));

    delete estimator;
    ssSetPWorkValue(S, 0, nullptr);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
