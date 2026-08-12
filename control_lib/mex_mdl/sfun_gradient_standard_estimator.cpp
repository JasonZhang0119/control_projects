#define S_FUNCTION_NAME sfun_gradient_standard_estimator
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <new>

#include "estimator/estimator_gradient.hpp"

#ifndef GSE_N_Y
#define GSE_N_Y 1
#endif

#ifndef GSE_N_THETA
#define GSE_N_THETA 2
#endif

using EstimatorType = GradientEstimator_Standard<real_T, GSE_N_Y, GSE_N_THETA>;

enum InputPortIndex
{
    INPUT_REGRESSOR = 0,
    INPUT_MEASUREMENT,
    INPUT_TS,
    INPUT_GAMMA,
    INPUT_PORT_COUNT
};

static void mdlInitializeSizes(SimStruct* S)
{
    ssSetNumSFcnParams(S, 0);

    if (!ssSetNumInputPorts(S, INPUT_PORT_COUNT))
    {
        return;
    }

    ssSetInputPortWidth(S, INPUT_REGRESSOR, GSE_N_THETA * GSE_N_Y);
    ssSetInputPortDataType(S, INPUT_REGRESSOR, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_REGRESSOR, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_REGRESSOR, 1);

    ssSetInputPortWidth(S, INPUT_MEASUREMENT, GSE_N_Y);
    ssSetInputPortDataType(S, INPUT_MEASUREMENT, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_MEASUREMENT, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_MEASUREMENT, 1);

    ssSetInputPortWidth(S, INPUT_TS, 1);
    ssSetInputPortDataType(S, INPUT_TS, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_TS, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_TS, 1);

    ssSetInputPortWidth(S, INPUT_GAMMA, 1);
    ssSetInputPortDataType(S, INPUT_GAMMA, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_GAMMA, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_GAMMA, 1);

    if (!ssSetNumOutputPorts(S, 1))
    {
        return;
    }

    ssSetOutputPortWidth(S, 0, GSE_N_THETA);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

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
        ssSetErrorStatus(S, "Failed to allocate GradientEstimator_Standard.");
        return;
    }

    ssSetPWorkValue(S, 0, estimator);
}

static bool ConfigureEstimatorFromInputs(SimStruct* S, EstimatorType& estimator)
{
    const real_T* regressor_input =
        static_cast<const real_T*>(ssGetInputPortSignal(S, INPUT_REGRESSOR));
    const real_T* measurement_input =
        static_cast<const real_T*>(ssGetInputPortSignal(S, INPUT_MEASUREMENT));
    const real_T* ts_input =
        static_cast<const real_T*>(ssGetInputPortSignal(S, INPUT_TS));
    const real_T* gamma_input =
        static_cast<const real_T*>(ssGetInputPortSignal(S, INPUT_GAMMA));

    EstimatorType::RegressorMatrix regressor;
    EstimatorType::MeasurementVector measurement;

    for (int column = 0; column < GSE_N_Y; ++column)
    {
        for (int row = 0; row < GSE_N_THETA; ++row)
        {
            const int input_index = row + column * GSE_N_THETA;
            regressor(row, column) = regressor_input[input_index];
        }
    }

    for (int index = 0; index < GSE_N_Y; ++index)
    {
        measurement(index) = measurement_input[index];
    }

    estimator.SetRegressor(regressor);
    estimator.SetMeasurement(measurement);
    estimator.SetTs(static_cast<real_T>(ts_input[0]));
    estimator.SetGamma(static_cast<real_T>(gamma_input[0]));

    return true;
}

static void CopyParameterOutput(SimStruct* S, const EstimatorType& estimator)
{
    real_T* parameter_output = static_cast<real_T*>(ssGetOutputPortSignal(S, 0));

    for (int index = 0; index < GSE_N_THETA; ++index)
    {
        parameter_output[index] = estimator.GetParameter()(index);
    }
}

static EstimatorType* GetEstimator(SimStruct* S)
{
    EstimatorType* estimator = static_cast<EstimatorType*>(ssGetPWorkValue(S, 0));

    if (estimator == nullptr)
    {
        ssSetErrorStatus(S, "GradientEstimator_Standard is not initialized.");
    }

    return estimator;
}

static void mdlOutputs(SimStruct* S, int_T tid)
{
    (void)tid;

    EstimatorType* estimator = GetEstimator(S);
    if (estimator == nullptr)
    {
        return;
    }

    EstimatorType output_estimator(*estimator);

    if (!ConfigureEstimatorFromInputs(S, output_estimator))
    {
        return;
    }

    output_estimator.Compute();
    CopyParameterOutput(S, output_estimator);
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct* S, int_T tid)
{
    (void)tid;

    EstimatorType* estimator = GetEstimator(S);
    if (estimator == nullptr)
    {
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
    EstimatorType* estimator = static_cast<EstimatorType*>(ssGetPWorkValue(S, 0));
    delete estimator;
    ssSetPWorkValue(S, 0, nullptr);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
