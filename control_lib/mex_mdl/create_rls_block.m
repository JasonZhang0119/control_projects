function blockPath = create_rls_block(modelName, blockName, initialCovariance)
%CREATE_RLS_BLOCK Add a configured Recursive Least Square S-Function block.

if nargin < 1 || isempty(modelName)
    modelName = 'recursive_least_square_example';
end

if nargin < 2 || isempty(blockName)
    blockName = 'Recursive Least Square';
end

if nargin < 3
    initialCovariance = 1000.0;
end

validateattributes(initialCovariance, {'double'}, ...
    {'real', 'scalar', 'positive', 'finite'});

if ~bdIsLoaded(modelName)
    new_system(modelName);
    open_system(modelName);
end

blockPath = [modelName, '/', blockName];

if getSimulinkBlockHandle(blockPath) == -1
    add_block('simulink/User-Defined Functions/S-Function', blockPath);
end

set_param(blockPath, ...
    'FunctionName', 'sfun_recursive_least_square', ...
    'Parameters', num2str(initialCovariance, 17));
end
