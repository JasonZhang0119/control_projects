function T = TransT(axis, d)
%TRANS Generate 4x4 translation matrix along a principal axis.
%
% Parameters
% ----------
% axis : char
%     Translation axis: 'x', 'y', or 'z'
% d : double
%     Translation distance
%
% Returns
% -------
% T : double (4x4)
%     Homogeneous transformation matrix
%
% Notes
% -----
% You only need to fill the corresponding position.

    T = eye(4);

    switch lower(axis)

        case 'x'
            T(1,4) = d;

        case 'y'
            T(2,4) = d;

        case 'z'
            T(3,4) = d;

        otherwise
            error('Trans:InvalidAxis', 'Axis must be x, y, or z');
    end
end