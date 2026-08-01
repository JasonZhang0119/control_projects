function result = run_parameter_estimator_model()
%RUN_PARAMETER_ESTIMATOR_MODEL Build, simulate, and verify the LS/RLS model.

testDirectory = fileparts(mfilename('fullpath'));
projectDirectory = fileparts(testDirectory);
mexModelDirectory = fullfile(projectDirectory, 'mex_mdl');
modelName = 'test_parameter_estimator_simulink';
modelFile = fullfile(testDirectory, [modelName, '.slx']);

addpath(testDirectory);
addpath(mexModelDirectory);

if (exist('sfun_recursive_least_square', 'file') ~= 3) || ...
        (exist('sfun_least_square', 'file') ~= 3)
    currentDirectory = pwd;
    cleanupDirectory = onCleanup(@() cd(currentDirectory));
    cd(mexModelDirectory);
    build_mex_mdl([], 2, 4);
end

if ~isfile(modelFile)
    create_parameter_estimator_model();
end

load_system(modelFile);
simulationOutput = sim(modelName, 'ReturnWorkspaceOutputs', 'on');

thetaRlsTimeseries = simulationOutput.get('theta_rls');
thetaLsTimeseries = simulationOutput.get('theta_ls');

thetaRls = reshape(thetaRlsTimeseries.Data(end, :), [], 1);
thetaLs = reshape(thetaLsTimeseries.Data(end, :), [], 1);
expectedTheta = [1.0; 2.0];

rlsError = norm(thetaRls - expectedTheta);
lsError = norm(thetaLs - expectedTheta);

assert(rlsError < 1.0e-3, ...
    'RLS Simulink test failed. Parameter error: %.6g', rlsError);
assert(lsError < 1.0e-10, ...
    'Batch LS Simulink test failed. Parameter error: %.6g', lsError);

result = struct( ...
    'RlsParameter', thetaRls, ...
    'LeastSquareParameter', thetaLs, ...
    'RlsError', rlsError, ...
    'LeastSquareError', lsError);

disp(result);
close_system(modelName, 0);
end
