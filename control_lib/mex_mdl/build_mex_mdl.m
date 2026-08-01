function build_mex_mdl(eigenInclude, parameterCount, stepCount)
%BUILD_MEX_MDL Build the RLS and batch LS Simulink S-functions.

thisDirectory = fileparts(mfilename('fullpath'));
projectDirectory = fileparts(thisDirectory);

if nargin < 1 || isempty(eigenInclude)
    eigenInclude = getenv('EIGEN3_INCLUDE_DIR');
end

if nargin < 2
    parameterCount = 2;
end

if nargin < 3
    stepCount = 4;
end

validateattributes(parameterCount, {'numeric'}, ...
    {'real', 'scalar', 'integer', 'positive', 'finite'});
validateattributes(stepCount, {'numeric'}, ...
    {'real', 'scalar', 'integer', 'positive', 'finite'});

if isempty(eigenInclude)
    defaultEigenInclude = 'D:\third_party_libs\cpp\eigen-5.0.0';
    if isfolder(defaultEigenInclude)
        eigenInclude = defaultEigenInclude;
    else
        error(['Pass the Eigen include directory to build_mex_mdl, or set ', ...
               'the EIGEN3_INCLUDE_DIR environment variable.']);
    end
end

recursiveSourceFile = fullfile( ...
    thisDirectory, 'sfun_recursive_least_square.cpp');
recursiveOutputFile = fullfile( ...
    thisDirectory, 'sfun_recursive_least_square');

mex('-v', ...
    ['-DRLS_N_THETA=', num2str(parameterCount)], ...
    ['-I', fullfile(projectDirectory, 'include')], ...
    ['-I', eigenInclude], ...
    recursiveSourceFile, ...
    '-output', recursiveOutputFile);

batchSourceFile = fullfile(thisDirectory, 'sfun_least_square.cpp');
batchOutputFile = fullfile(thisDirectory, 'sfun_least_square');

mex('-v', ...
    ['-DLS_N_THETA=', num2str(parameterCount)], ...
    ['-DLS_N_STEP=', num2str(stepCount)], ...
    ['-I', fullfile(projectDirectory, 'include')], ...
    ['-I', eigenInclude], ...
    batchSourceFile, ...
    '-output', batchOutputFile);
end
