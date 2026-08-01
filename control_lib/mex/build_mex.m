function build_mex(eigenInclude, parameterCount)
%BUILD_MEX Build the object-oriented Recursive Least Square MEX interface.

thisDirectory = fileparts(mfilename('fullpath'));
projectDirectory = fileparts(thisDirectory);

if nargin < 1 || isempty(eigenInclude)
    eigenInclude = getenv('EIGEN3_INCLUDE_DIR');
end

if nargin < 2
    parameterCount = 2;
end

validateattributes(parameterCount, {'numeric'}, ...
    {'real', 'scalar', 'integer', 'positive', 'finite'});

if isempty(eigenInclude)
    defaultEigenInclude = 'D:\third_party_libs\cpp\eigen-5.0.0';
    if isfolder(defaultEigenInclude)
        eigenInclude = defaultEigenInclude;
    else
        error(['Pass the Eigen include directory to build_mex, or set ', ...
               'the EIGEN3_INCLUDE_DIR environment variable.']);
    end
end

sourceFile = fullfile(thisDirectory, 'recursive_least_square_mex.cpp');
outputFile = fullfile(thisDirectory, 'recursive_least_square_mex');

mex('-R2018a', '-v', ...
    ['-DRLS_N_THETA=', num2str(parameterCount)], ...
    ['-I', fullfile(projectDirectory, 'include')], ...
    ['-I', eigenInclude], ...
    sourceFile, ...
    '-output', outputFile);
end
