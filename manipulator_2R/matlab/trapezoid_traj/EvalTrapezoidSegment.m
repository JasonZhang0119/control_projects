function [q, q_dot, q_ddot] = EvalTrapezoidSegment(seg, t)
%EVALTRAPEZOIDSEGMENT Template for evaluating one trapezoidal segment.
%
% Inputs
% ------
% seg : struct
%     Segment structure created by CreateMultiSegmentTrapezoidal.
%
% t : double
%     Query time.
%
% Outputs
% -------
% q, q_dot, q_ddot : double
%     Position, velocity, and acceleration at time t.

    if ~isfield(seg, 't0') || ~isfield(seg, 'tf')
        error('EvalTrapezoidSegment:InvalidSegment', ...
              'Segment must contain t0 and tf.');
    end

    if t <= seg.t0
        tau = 0;
    elseif t >= seg.tf
        tau = seg.tf - seg.t0;
    else
        tau = t - seg.t0;
    end

    % ============================================================
    % Step 1: Read the segment profile
    % ============================================================
    %
    % TODO:
    %   Extract the profile parameters you store in seg.profile.
    %   Suggested fields may include:
    %       t_acc, t_flat, t_dec, v_peak, a_max, direction, distance
    %
    %   Example:
    %       profile = seg.profile;
    %
    profile = seg.profile; %#ok<NASGU>

    % ============================================================
    % Step 2: Evaluate the motion phase
    % ============================================================
    %
    % TODO:
    %   Decide whether tau is in:
    %       - acceleration phase
    %       - cruise phase
    %       - deceleration phase
    %
    %   Then compute:
    %       q(t), q_dot(t), q_ddot(t)
    %
    %   Common approach:
    %       1) Compute local scalar progress s(t)
    %       2) Map s(t) to joint vector along the segment direction
    %
    q = [];
    q_dot = [];
    q_ddot = [];

    % TODO: Replace the placeholders above with the real formula.
end
