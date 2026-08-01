#include "parameter_estimator.hpp"

#include <iostream>
#include <stdexcept>


int main()
{
    using Estimator = LeastSquareEstimator_WeightBatch<double, 2, 4>;

    Estimator estimator;

    Estimator::RegressorMatrix regressor;
    regressor << 1.0, 0.0,
                 1.0, 1.0,
                 1.0, 2.0,
                 1.0, 3.0;

    Estimator::MeasurementVector measurement;
    measurement << 1.0, 3.0, 5.0, 9.0;

    Estimator::ParameterVector expected_unweighted_parameter;
    expected_unweighted_parameter << 0.6, 2.6;

    Estimator::ParameterVector expected_weighted_parameter;
    expected_weighted_parameter << 0.2, 2.8;

    Estimator::WeightVector weight;
    weight << 1.0, 2.0, 3.0, 4.0;

    estimator.SetRegressor(regressor);
    estimator.SetMeasurement(measurement);
    estimator.Compute();

    if (!estimator.GetParameter().isApprox(expected_unweighted_parameter, 1e-12))
    {
        std::cerr << "Unweighted parameter estimation failed.\n"
                  << "Expected:\n" << expected_unweighted_parameter << "\n"
                  << "Actual:\n" << estimator.GetParameter() << '\n';
        return 1;
    }

    estimator.SetWeightVector(weight);
    estimator.Compute();

    if (!estimator.GetParameter().isApprox(expected_weighted_parameter, 1e-12))
    {
        std::cerr << "Weighted parameter estimation failed.\n"
                  << "Expected:\n" << expected_weighted_parameter << "\n"
                  << "Actual:\n" << estimator.GetParameter() << '\n';
        return 1;
    }

    Estimator::WeightVector invalid_weight = weight;
    invalid_weight(0) = -1.0;

    try
    {
        estimator.SetWeightVector(invalid_weight);
        std::cerr << "Negative weight did not throw std::invalid_argument.\n";
        return 1;
    }
    catch (const std::invalid_argument&)
    {
    }

    estimator.Reset();

    if (!estimator.GetRegressor().isZero() ||
        !estimator.GetMeasurement().isZero() ||
        !estimator.GetParameter().isZero() ||
        !estimator.GetWeight().isOnes())
    {
        std::cerr << "Reset failed.\n";
        return 1;
    }

    using RecursiveEstimator = LeastSquareEstimator_Recursive<double, 2>;

    RecursiveEstimator recursive_estimator;
    RecursiveEstimator::RegressorVector recursive_regressor;
    recursive_regressor << 1.0, 2.0;

    RecursiveEstimator::ParameterVector initial_parameter;
    initial_parameter << 0.5, 1.5;

    const RecursiveEstimator::CovarianceMatrix initial_covariance =
        RecursiveEstimator::CovarianceMatrix::Identity() * 1000.0;

    recursive_estimator.SetRegressor(recursive_regressor);
    recursive_estimator.SetMeasurement(5.0);
    recursive_estimator.SetParameter(initial_parameter);
    recursive_estimator.SetCovariance(initial_covariance);

    if (!recursive_estimator.SetForgettingFactor(0.99) ||
        recursive_estimator.SetForgettingFactor(0.0))
    {
        std::cerr << "Recursive estimator forgetting-factor validation failed.\n";
        return 1;
    }

    if (!recursive_estimator.GetRegressor().isApprox(recursive_regressor) ||
        !recursive_estimator.GetParameter().isApprox(initial_parameter) ||
        !recursive_estimator.GetCovariance().isApprox(initial_covariance) ||
        (recursive_estimator.GetMeasurement() != 5.0))
    {
        std::cerr << "Recursive estimator state setup failed.\n";
        return 1;
    }

    recursive_estimator.Reset();

    if (!recursive_estimator.GetRegressor().isZero() ||
        !recursive_estimator.GetParameter().isZero() ||
        !recursive_estimator.GetGain().isZero() ||
        !recursive_estimator.GetCovariance().isIdentity() ||
        (recursive_estimator.GetMeasurement() != 0.0) ||
        (recursive_estimator.GetInnovation() != 0.0) ||
        (recursive_estimator.GetForgettingFactor() != 1.0))
    {
        std::cerr << "Recursive estimator reset failed.\n";
        return 1;
    }

    std::cout << "All parameter estimator tests passed.\n";
    return 0;
}
