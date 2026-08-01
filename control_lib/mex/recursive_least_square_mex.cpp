#include "mex.hpp"
#include "mexAdapter.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "parameter_estimator.hpp"

#ifndef RLS_N_THETA
#define RLS_N_THETA 2
#endif

class MexFunction : public matlab::mex::Function
{
private:
    typedef LeastSquareEstimator_Recursive<double, RLS_N_THETA> EstimatorType;
    typedef std::unordered_map<std::uint64_t, std::unique_ptr<EstimatorType>> EstimatorMap;

public:
    MexFunction()
        : matlab_engine_(getEngine())
    {
    }

    void operator()(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs) override
    {
        if ((inputs.size() < 1) ||
            (inputs[0].getType() != matlab::data::ArrayType::CHAR))
        {
            RaiseError("The first input must be a command character vector.");
        }

        const matlab::data::CharArray command_array = inputs[0];
        const std::string command = command_array.toAscii();

        if (command == "create")
        {
            Create(outputs, inputs);
        }
        else if (command == "destroy")
        {
            Destroy(outputs, inputs);
        }
        else if (command == "reset")
        {
            Reset(outputs, inputs);
        }
        else if (command == "step")
        {
            Step(outputs, inputs);
        }
        else if (command == "get_state")
        {
            GetState(outputs, inputs);
        }
        else if (command == "set_parameter")
        {
            SetParameter(outputs, inputs);
        }
        else if (command == "set_covariance")
        {
            SetCovariance(outputs, inputs);
        }
        else if (command == "parameter_count")
        {
            ParameterCount(outputs, inputs);
        }
        else
        {
            RaiseError("Unknown command: " + command);
        }
    }

private:
    void Create(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        if (outputs.size() != 1)
        {
            RaiseError("create requires exactly one output handle.");
        }

        if (inputs.size() > 2)
        {
            RaiseError("Usage: handle = recursive_least_square_mex('create', initialCovariance).");
        }

        double initial_covariance = 1.0;
        if (inputs.size() == 2)
        {
            initial_covariance = ReadPositiveScalar(inputs[1], "initialCovariance");
        }

        std::unique_ptr<EstimatorType> estimator(new EstimatorType());
        SetInitialCovariance(*estimator, initial_covariance);

        const std::uint64_t handle = next_handle_;
        ++next_handle_;

        estimators_.emplace(handle, std::move(estimator));
        mexLock();
        outputs[0] = factory_.createScalar(handle);
    }

    void Destroy(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireNoOutput(outputs, "destroy");
        RequireInputCount(inputs, 2, "destroy");

        const std::uint64_t handle = ReadHandle(inputs[1]);
        EstimatorMap::iterator iterator = estimators_.find(handle);

        if (iterator == estimators_.end())
        {
            RaiseError("Invalid or already destroyed estimator handle.");
        }

        estimators_.erase(iterator);
        mexUnlock();
    }

    void Reset(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireNoOutput(outputs, "reset");

        if ((inputs.size() < 2) || (inputs.size() > 3))
        {
            RaiseError("Usage: recursive_least_square_mex('reset', handle, initialCovariance).");
        }

        EstimatorType& estimator = FindEstimator(inputs[1]);
        double initial_covariance = 1.0;

        if (inputs.size() == 3)
        {
            initial_covariance = ReadPositiveScalar(inputs[2], "initialCovariance");
        }

        estimator.Reset();
        SetInitialCovariance(estimator, initial_covariance);
    }

    void Step(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireInputCount(inputs, 5, "step");

        if (outputs.size() > 4)
        {
            RaiseError("step returns at most four outputs: theta, gain, innovation, covariance.");
        }

        EstimatorType& estimator = FindEstimator(inputs[1]);
        const EstimatorType::RegressorVector regressor = ReadVector(inputs[2], "regressor");
        const double measurement = ReadScalar(inputs[3], "measurement");
        const double forgetting_factor = ReadScalar(inputs[4], "forgettingFactor");

        if (!estimator.SetForgettingFactor(forgetting_factor))
        {
            RaiseError("forgettingFactor must be in the interval (0, 1].");
        }

        estimator.SetRegressor(regressor);
        estimator.SetMeasurement(measurement);
        estimator.Compute();
        WriteState(outputs, estimator);
    }

    void GetState(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireInputCount(inputs, 2, "get_state");

        if (outputs.size() > 4)
        {
            RaiseError("get_state returns at most four outputs: theta, gain, innovation, covariance.");
        }

        const EstimatorType& estimator = FindEstimator(inputs[1]);
        WriteState(outputs, estimator);
    }

    void SetParameter(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireNoOutput(outputs, "set_parameter");
        RequireInputCount(inputs, 3, "set_parameter");

        EstimatorType& estimator = FindEstimator(inputs[1]);
        const EstimatorType::ParameterVector parameter = ReadVector(inputs[2], "parameter");
        estimator.SetParameter(parameter);
    }

    void SetCovariance(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireNoOutput(outputs, "set_covariance");
        RequireInputCount(inputs, 3, "set_covariance");

        EstimatorType& estimator = FindEstimator(inputs[1]);
        const EstimatorType::CovarianceMatrix covariance =
            ReadCovariance(inputs[2], "covariance");
        estimator.SetCovariance(covariance);
    }

    void ParameterCount(
        matlab::mex::ArgumentList outputs,
        matlab::mex::ArgumentList inputs)
    {
        RequireInputCount(inputs, 1, "parameter_count");

        if (outputs.size() != 1)
        {
            RaiseError("parameter_count requires exactly one output.");
        }

        outputs[0] = factory_.createScalar(static_cast<double>(RLS_N_THETA));
    }

    void WriteState(
        matlab::mex::ArgumentList outputs,
        const EstimatorType& estimator)
    {
        if (outputs.size() >= 1)
        {
            outputs[0] = CreateVector(estimator.GetParameter());
        }

        if (outputs.size() >= 2)
        {
            outputs[1] = CreateVector(estimator.GetGain());
        }

        if (outputs.size() >= 3)
        {
            outputs[2] = factory_.createScalar(estimator.GetInnovation());
        }

        if (outputs.size() >= 4)
        {
            outputs[3] = CreateCovariance(estimator.GetCovariance());
        }
    }

    matlab::data::TypedArray<double> CreateVector(
        const EstimatorType::ParameterVector& vector)
    {
        matlab::data::TypedArray<double> output =
            factory_.createArray<double>({static_cast<std::size_t>(RLS_N_THETA), 1});
        matlab::data::TypedArray<double>::iterator output_iterator = output.begin();

        for (int index = 0; index < RLS_N_THETA; ++index)
        {
            *output_iterator = vector(index);
            ++output_iterator;
        }

        return output;
    }

    matlab::data::TypedArray<double> CreateCovariance(
        const EstimatorType::CovarianceMatrix& covariance)
    {
        matlab::data::TypedArray<double> output =
            factory_.createArray<double>({
                static_cast<std::size_t>(RLS_N_THETA),
                static_cast<std::size_t>(RLS_N_THETA)});
        matlab::data::TypedArray<double>::iterator output_iterator = output.begin();

        for (int column = 0; column < RLS_N_THETA; ++column)
        {
            for (int row = 0; row < RLS_N_THETA; ++row)
            {
                *output_iterator = covariance(row, column);
                ++output_iterator;
            }
        }

        return output;
    }

    EstimatorType::RegressorVector ReadVector(
        const matlab::data::Array& input,
        const std::string& name)
    {
        if ((input.getType() != matlab::data::ArrayType::DOUBLE) ||
            (input.getNumberOfElements() != static_cast<std::size_t>(RLS_N_THETA)))
        {
            RaiseError(name + " must be a real double vector with RLS_N_THETA elements.");
        }

        const matlab::data::TypedArray<double> values = input;
        matlab::data::TypedArray<double>::const_iterator input_iterator = values.cbegin();
        EstimatorType::RegressorVector vector;

        for (int index = 0; index < RLS_N_THETA; ++index)
        {
            vector(index) = *input_iterator;
            ++input_iterator;
        }

        return vector;
    }

    EstimatorType::CovarianceMatrix ReadCovariance(
        const matlab::data::Array& input,
        const std::string& name)
    {
        const std::size_t expected_elements =
            static_cast<std::size_t>(RLS_N_THETA * RLS_N_THETA);

        if ((input.getType() != matlab::data::ArrayType::DOUBLE) ||
            (input.getNumberOfElements() != expected_elements))
        {
            RaiseError(name + " must be a real double RLS_N_THETA-by-RLS_N_THETA matrix.");
        }

        const matlab::data::TypedArray<double> values = input;
        matlab::data::TypedArray<double>::const_iterator input_iterator = values.cbegin();
        EstimatorType::CovarianceMatrix covariance;

        for (int column = 0; column < RLS_N_THETA; ++column)
        {
            for (int row = 0; row < RLS_N_THETA; ++row)
            {
                covariance(row, column) = *input_iterator;
                ++input_iterator;
            }
        }

        return covariance;
    }

    double ReadScalar(
        const matlab::data::Array& input,
        const std::string& name)
    {
        if ((input.getType() != matlab::data::ArrayType::DOUBLE) ||
            (input.getNumberOfElements() != 1))
        {
            RaiseError(name + " must be a real double scalar.");
        }

        const matlab::data::TypedArray<double> value = input;
        return *(value.cbegin());
    }

    double ReadPositiveScalar(
        const matlab::data::Array& input,
        const std::string& name)
    {
        const double value = ReadScalar(input, name);

        if (value <= 0.0)
        {
            RaiseError(name + " must be greater than zero.");
        }

        return value;
    }

    std::uint64_t ReadHandle(const matlab::data::Array& input)
    {
        if ((input.getType() != matlab::data::ArrayType::UINT64) ||
            (input.getNumberOfElements() != 1))
        {
            RaiseError("Estimator handle must be a uint64 scalar.");
        }

        const matlab::data::TypedArray<std::uint64_t> handle_array = input;
        return *(handle_array.cbegin());
    }

    EstimatorType& FindEstimator(const matlab::data::Array& handle_input)
    {
        const std::uint64_t handle = ReadHandle(handle_input);
        EstimatorMap::iterator iterator = estimators_.find(handle);

        if (iterator == estimators_.end())
        {
            RaiseError("Invalid or already destroyed estimator handle.");
        }

        return *(iterator->second);
    }

    void SetInitialCovariance(
        EstimatorType& estimator,
        double initial_covariance)
    {
        const EstimatorType::CovarianceMatrix covariance =
            EstimatorType::CovarianceMatrix::Identity() * initial_covariance;
        estimator.SetCovariance(covariance);
    }

    void RequireInputCount(
        matlab::mex::ArgumentList& inputs,
        std::size_t expected_count,
        const std::string& command)
    {
        if (inputs.size() != expected_count)
        {
            RaiseError(command + " received an incorrect number of inputs.");
        }
    }

    void RequireNoOutput(
        matlab::mex::ArgumentList& outputs,
        const std::string& command)
    {
        if (!outputs.empty())
        {
            RaiseError(command + " does not return an output.");
        }
    }

    [[noreturn]] void RaiseError(const std::string& message)
    {
        std::vector<matlab::data::Array> arguments;
        arguments.push_back(factory_.createCharArray(message));
        matlab_engine_->feval(u"error", 0, arguments);
        throw std::runtime_error(message);
    }

private:
    matlab::data::ArrayFactory factory_;
    std::shared_ptr<matlab::engine::MATLABEngine> matlab_engine_;
    EstimatorMap estimators_;
    std::uint64_t next_handle_{1};
};
