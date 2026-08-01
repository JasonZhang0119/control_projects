#define S_FUNCTION_NAME sfun_least_square
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include "parameter_estimator.hpp"

#ifndef LS_N_THETA
#define LS_N_THETA 2
#endif

#ifndef LS_N_STEP
#define LS_N_STEP 4
#endif

typedef LeastSquareEstimator_WeightBatch<real_T, LS_N_THETA, LS_N_STEP> EstimatorType;

enum InputPortIndex
{
    INPUT_REGRESSOR = 0,
    INPUT_MEASUREMENT,
    INPUT_WEIGHT,
    INPUT_PORT_COUNT
};

static void mdlInitializeSizes(SimStruct* S)
{
    ssSetNumSFcnParams(S, 0);

    if (!ssSetNumInputPorts(S, INPUT_PORT_COUNT))
    {
        return;
    }

    ssSetInputPortWidth(S, INPUT_REGRESSOR, LS_N_STEP * LS_N_THETA);
    ssSetInputPortDataType(S, INPUT_REGRESSOR, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_REGRESSOR, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_REGRESSOR, 1);

    ssSetInputPortWidth(S, INPUT_MEASUREMENT, LS_N_STEP);
    ssSetInputPortDataType(S, INPUT_MEASUREMENT, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_MEASUREMENT, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_MEASUREMENT, 1);

    ssSetInputPortWidth(S, INPUT_WEIGHT, LS_N_STEP);
    ssSetInputPortDataType(S, INPUT_WEIGHT, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, INPUT_WEIGHT, 1);
    ssSetInputPortRequiredContiguous(S, INPUT_WEIGHT, 1);

    if (!ssSetNumOutputPorts(S, 1))
    {
        return;
    }

    ssSetOutputPortWidth(S, 0, LS_N_THETA);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_WORKS_WITH_CODE_REUSE);
}

static void mdlInitializeSampleTimes(SimStruct* S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct* S, int_T tid)
{
    (void)tid;

    const real_T* regressor_input = static_cast<const real_T*>(
        ssGetInputPortSignal(S, INPUT_REGRESSOR));
    const real_T* measurement_input = static_cast<const real_T*>(
        ssGetInputPortSignal(S, INPUT_MEASUREMENT));
    const real_T* weight_input = static_cast<const real_T*>(
        ssGetInputPortSignal(S, INPUT_WEIGHT));
    real_T* parameter_output =
        static_cast<real_T*>(ssGetOutputPortSignal(S, 0));

    EstimatorType::RegressorMatrix regressor;
    EstimatorType::MeasurementVector measurement;
    EstimatorType::WeightVector weight;

    for (int column = 0; column < LS_N_THETA; ++column)
    {
        for (int row = 0; row < LS_N_STEP; ++row)
        {
            const int input_index = row + column * LS_N_STEP;
            regressor(row, column) = regressor_input[input_index];
        }
    }

    for (int row = 0; row < LS_N_STEP; ++row)
    {
        if (weight_input[row] < 0.0)
        {
            ssSetErrorStatus(S, "Least Square weights must be non-negative.");
            return;
        }

        measurement(row) = measurement_input[row];
        weight(row) = weight_input[row];
    }

    EstimatorType estimator;
    estimator.SetRegressor(regressor);
    estimator.SetMeasurement(measurement);
    estimator.SetWeightVector(weight);
    estimator.Compute();

    for (int index = 0; index < LS_N_THETA; ++index)
    {
        parameter_output[index] = estimator.GetParameter()(index);
    }
}

static void mdlTerminate(SimStruct* S)
{
    (void)S;
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
