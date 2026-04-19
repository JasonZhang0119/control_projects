%% Baseline controller settings
kP_original = 0.02;
kD_original = 0.5/360;

s = tf('s');

C_original = kP_original + kD_original*s;

% Open-loop transfer function (baseline)
L_original = C_original * sys_full_degree_tf;

% Plant and baseline loop margins
figure
margin(sys_full_degree_tf);
figure
margin(L_original);

% figure
% nyquist(L_original)

grid on

%% Tuned controller settings
kP = 0.04;
kD = 0.04 * 0.0015;

C_tune = kP + kD *s;

% Open-loop transfer function (tuned)
L_tune = C_tune * sys_full_degree_tf;

% Compare baseline vs tuned margins
figure
margin(L_original);
hold on
margin(L_tune);

%% Compare Nyquist curves
figure
% nyquist(L_original);
% hold on
nyquist(L_tune)

grid on

%% Closed-loop poles/zeros (tuned controller)
zpk(feedback(L_tune, 1))
