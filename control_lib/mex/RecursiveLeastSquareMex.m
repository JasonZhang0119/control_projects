classdef RecursiveLeastSquareMex < handle
    properties (SetAccess = private)
        ParameterCount (1, 1) double = 0
    end

    properties (SetAccess = private, Hidden)
        Handle (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = RecursiveLeastSquareMex(initialCovariance)
            if nargin < 1
                initialCovariance = 1.0;
            end

            validateattributes(initialCovariance, {'double'}, ...
                {'real', 'scalar', 'positive', 'finite'});

            obj.ParameterCount = recursive_least_square_mex('parameter_count');
            obj.Handle = recursive_least_square_mex( ...
                'create', initialCovariance);
        end

        function delete(obj)
            if obj.Handle ~= uint64(0)
                try
                    recursive_least_square_mex('destroy', obj.Handle);
                catch
                    % MATLAB may already be unloading the MEX file.
                end
                obj.Handle = uint64(0);
            end
        end

        function [theta, gain, innovation, covariance] = step( ...
                obj, regressor, measurement, forgettingFactor)
            if nargin < 4
                forgettingFactor = 1.0;
            end

            validateattributes(regressor, {'double'}, ...
                {'real', 'vector', 'numel', obj.ParameterCount, 'finite'});
            validateattributes(measurement, {'double'}, ...
                {'real', 'scalar', 'finite'});
            validateattributes(forgettingFactor, {'double'}, ...
                {'real', 'scalar', '>', 0.0, '<=', 1.0, 'finite'});

            [theta, gain, innovation, covariance] = ...
                recursive_least_square_mex( ...
                    'step', obj.Handle, regressor(:), ...
                    measurement, forgettingFactor);
        end

        function reset(obj, initialCovariance)
            if nargin < 2
                initialCovariance = 1.0;
            end

            validateattributes(initialCovariance, {'double'}, ...
                {'real', 'scalar', 'positive', 'finite'});
            recursive_least_square_mex( ...
                'reset', obj.Handle, initialCovariance);
        end

        function setParameter(obj, parameter)
            validateattributes(parameter, {'double'}, ...
                {'real', 'vector', 'numel', obj.ParameterCount, 'finite'});
            recursive_least_square_mex( ...
                'set_parameter', obj.Handle, parameter(:));
        end

        function setCovariance(obj, covariance)
            validateattributes(covariance, {'double'}, ...
                {'real', 'size', [obj.ParameterCount, obj.ParameterCount], 'finite'});
            recursive_least_square_mex( ...
                'set_covariance', obj.Handle, covariance);
        end

        function [theta, gain, innovation, covariance] = getState(obj)
            [theta, gain, innovation, covariance] = ...
                recursive_least_square_mex('get_state', obj.Handle);
        end
    end
end
