function T = TransR(axis, theta)
%TRANSR Homogeneous rotation matrix.
%
% Returns
% -------
% T : (4x4)

    T = eye(4);

    ct = cos(theta);
    st = sin(theta);

    switch lower(axis)

        case 'x'
            T = [1, 0, 0, 0;...
                 0,  ct, -st, 0;...
                 0,  st, ct, 0;...
                 0,    0, 0, 1];

        case 'y'
            T = [ct, 0, st, 0;...
                 0,  1, 0, 0;...
                 -st,  0, ct, 0;...
                 0,    0, 0, 1];

        case 'z'
            T = [ct, -st, 0, 0;...
                 st,  ct, 0, 0;...
                 0,    0, 1, 0;...
                 0,    0, 0, 1];

        otherwise
            error('TransR:InvalidAxis', 'Axis must be x, y, or z.');
    end
end