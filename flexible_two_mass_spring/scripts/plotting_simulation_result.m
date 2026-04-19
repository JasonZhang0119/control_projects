clc;
close all;

%% Basic check
if ~exist('out','var')
    error('SimulationOutput variable "out" not found in workspace.');
end

%% Extract simulation data
t = out.tout;

phi2 = squeeze(out.phi_2.signals.values);
phiRef = squeeze(out.phi_ref.signals.values);
phiRefF = squeeze(out.phi_ref_filtered.signals.values);
u = squeeze(out.u.signals.values);
i = squeeze(out.i.signals.values);

%% Unit conversion (kept as-is for compatibility)
deg = 1;
phi2 = phi2 * deg;
phiRef = phiRef * deg;
phiRefF = phiRefF * deg;

%% Plot results
fs = 14;
figure('Color','w','Position',[100 100 900 600]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% Subplot 1: angle tracking
ax1 = nexttile;
plot(t, phiRef,'k--','LineWidth',2); hold on;
plot(t, phi2,'b','LineWidth',1.8);
% plot(t, phiRefF,'r--','LineWidth',1.8);
grid on;
ylabel('Angle [deg]','FontSize',fs);
title('Angle Tracking','FontSize',fs);
legend('$\varphi_{ref}$','$\varphi_{2}$','$\varphi_{ref,\mathrm{filtered}}$', ...
    'Location','best','Interpreter','latex','FontSize',fs);
set(ax1,'FontSize',fs);

% Subplot 2: motor voltage and current
ax2 = nexttile;
plot(t, u,'b-','LineWidth',1.8); hold on;
plot(t, i,'r-','LineWidth',1.8);
ylabel('Magnitude','FontSize',fs);
legend('$u$ (Voltage)','$i$ (Current)', ...
    'Location','best','Interpreter','latex','FontSize',fs);

grid on;
xlabel('Time [s]','FontSize',fs);
title('Motor Voltage and Current (Shared Axis)','FontSize',fs);
set(ax2,'FontSize',fs);

% Link x-axes for synchronized zoom/pan
linkaxes([ax1 ax2],'x');
