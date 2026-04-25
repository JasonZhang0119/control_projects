function q = IK2R_Continuous(p, param, q_prev)
%IK2R_CONTINUOUS Continuous inverse kinematics for a planar 2R robot.
%
% This template keeps the branch-selection logic and leaves the core
% analytic inverse-kinematics part as TODOs for you to fill in.
%
% Inputs
% ------
% p : double, size (2,1) or (1,2)
%     End-effector position [x; y]
%
% param : struct
%     Robot parameter struct. Expected fields:
%         l1, l2
%
% q_prev : double, size (2,1) or (1,2), optional
%     Previous joint solution. If empty, the default branch is returned.
%
% Output
% ------
% q : double, size (2,1)
%     Selected continuous IK solution.

    if nargin < 3
        q_prev = [];
    end

    p = p(:);

    if numel(p) ~= 2
        error('IK2R_Continuous:DimensionMismatch', ...
              'p must be a 2-element position vector.');
    end

    x = p(1);
    y = p(2);

    l1 = param.l1;
    l2 = param.l2;

    % ============================================================
    % Step 1: Compute the two analytic IK branches
    % ============================================================
    %
    % TODO:
    %   Use the standard 2R planar IK formulas to compute:
    %     q_a = [q1_a; q2_a]
    %     q_b = [q1_b; q2_b]
    %
    %   Common steps:
    %     c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2)
    %     s2 = +/- sqrt(1 - c2^2)
    %     q2 = atan2(s2, c2)
    %     q1 = atan2(y, x) - atan2(l2*sin(q2), l1 + l2*cos(q2))
    %
    %   Remember to wrap angles to [-pi, pi] if needed.
    %
    q_a = [NaN; NaN];
    q_b = [NaN; NaN];

    % Example placeholder structure:
    % c2 = ...
    % s2_a = ...
    % s2_b = ...
    % q2_a = ...
    % q2_b = ...
    % q1_a = ...
    % q1_b = ...
    % q_a = [q1_a; q2_a];
    % q_b = [q1_b; q2_b];

    % ============================================================
    % Step 2: Select the branch closest to q_prev
    % ============================================================
    %
    % TODO:
    %   If q_prev is empty, return your preferred default branch.
    %   Otherwise, choose the branch with smaller wrapped angle distance
    %   to q_prev.
    %
    % Suggested helper:
    %   dist_a = norm(angleDiffVec(q_a, q_prev));
    %   dist_b = norm(angleDiffVec(q_b, q_prev));
    %
    % q = ...

    if isempty(q_prev)
        q = q_a;  % TODO: choose your default branch here
    else
        q_prev = q_prev(:);

        if numel(q_prev) ~= 2
            error('IK2R_Continuous:PrevDimensionMismatch', ...
                  'q_prev must be a 2-element joint vector.');
        end

        % TODO: compute wrapped distance and select the closer branch
        % dist_a = ...
        % dist_b = ...
        % if dist_a <= dist_b
        %     q = q_a;
        % else
        %     q = q_b;
        % end

        q = q_a;  % TODO: replace with branch selection logic
    end
end

% -------------------------------------------------------------------------
% Local helper: wrap angle to [-pi, pi]
% -------------------------------------------------------------------------
function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

% -------------------------------------------------------------------------
% Local helper: wrapped angle difference
% -------------------------------------------------------------------------
function d = angleDiffVec(a, b)
    d = [wrapToPiLocal(a(1) - b(1)); wrapToPiLocal(a(2) - b(2))];
end
