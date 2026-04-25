function [q, info] = IK2R_Numerical(p, param, options)
%IK2R_NUMERICAL Numerical inverse kinematics template for a planar 2R robot.
%
% This is a scaffold for trying numerical IK methods such as:
%   - Newton-Raphson
%   - Jacobian pseudo-inverse
%   - Jacobian transpose
%   - Damped least squares
%
% Inputs
% ------
% p : double, size (2,1) or (1,2)
%     Target end-effector position [x; y].
%
% param : struct
%     Robot parameters. Expected fields:
%         l1, l2
%
% options : struct, optional
%     Numerical solver settings:
%         q0          : initial guess, size (2,1)
%         max_iter    : maximum iteration count
%         tol         : position error tolerance
%         step_size   : step size / gain
%         method      : 'pinv', 'transpose', or 'dls'
%         lambda      : damping factor for DLS
%
% Outputs
% -------
% q : double, size (2,1)
%     Estimated joint solution.
%
% info : struct
%     Solver diagnostics:
%         success
%         iterations
%         final_error
%         method

    if nargin < 3
        options = struct();
    end

    p = p(:);

    if numel(p) ~= 2
        error('IK2R_Numerical:DimensionMismatch', ...
              'p must be a 2-element position vector.');
    end

    if ~isfield(options, 'q0')
        options.q0 = [0; 0];
    end

    if ~isfield(options, 'max_iter')
        options.max_iter = 100;
    end

    if ~isfield(options, 'tol')
        options.tol = 1e-8;
    end

    if ~isfield(options, 'step_size')
        options.step_size = 1.0;
    end

    if ~isfield(options, 'method')
        options.method = 'pinv';
    end

    if ~isfield(options, 'lambda')
        options.lambda = 1e-3;
    end

    q = options.q0(:);

    if numel(q) ~= 2
        error('IK2R_Numerical:InitialGuessMismatch', ...
              'options.q0 must be a 2-element joint vector.');
    end

    info.success = false;
    info.iterations = 0;
    info.final_error = NaN;
    info.method = options.method;

    % ============================================================
    % Iterative solver loop
    % ============================================================
    for iter = 1:options.max_iter
        p_hat = FK2R_Analytic(q, param);
        e = p - p_hat;

        err_norm = norm(e);
        info.iterations = iter;
        info.final_error = err_norm;

        if err_norm < options.tol
            info.success = true;
            break;
        end

        J = Jacobian2R_Analytic(q, param);

        % --------------------------------------------------------
        % TODO: choose the update law you want to study
        %
        % Common options:
        %   1) Newton / pseudo-inverse:
        %        dq = J \ e
        %        dq = pinv(J) * e
        %
        %   2) Jacobian transpose:
        %        dq = J' * e
        %
        %   3) Damped least squares:
        %        dq = J' * inv(J*J' + lambda^2*I) * e
        %
        % Then update:
        %        q = q + step_size * dq
        % --------------------------------------------------------

        dq = zeros(2, 1);  % TODO: replace with your chosen update rule

        % TODO: optionally wrap the joint angles after each step
        % q = wrapToPiLocal(q + options.step_size * dq);

        q = q + options.step_size * dq;
    end

    % ============================================================
    % Optional post-processing
    % ============================================================
    % TODO:
    %   You may want to clamp q, wrap angles, or re-evaluate the final
    %   forward kinematics to report a more detailed residual.

    if ~info.success
        info.final_error = norm(p - FK2R_Analytic(q, param));
    end
end

% -------------------------------------------------------------------------
% Local helper: wrap angle to [-pi, pi]
% -------------------------------------------------------------------------
function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
