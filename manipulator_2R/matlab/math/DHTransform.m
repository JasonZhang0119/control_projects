function T = DHTransform(a, alpha, d, theta, method)
%DHTRANSFORM Homogeneous transform using DH method.
%
% Parameters
% ----------
% a : double
% alpha : double
% d : double
% theta : double
% method : char
%     'standard' or 'modified'
%
% Returns
% -------
% T : double (4x4)
%
% Notes
% -----
% standard:
%   A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
%
% modified (Craig):
%   A = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)

    if nargin < 5
        method = 'standard';
    end

    switch lower(method)

        case 'standard'
            T = TransR('z', theta) * ...
                TransT('z', d) * ...
                TransT('x', a) * ...
                TransR('x', alpha);

        case 'craig'
            T = TransR('x', alpha) * ...
                TransT('x', a) * ...
                TransR('z', theta) * ...
                TransT('z', d);

        otherwise
            error('DHTransform:UnknownMethod', ...
                  'method must be ''standard'' or ''modified''.');
    end
end