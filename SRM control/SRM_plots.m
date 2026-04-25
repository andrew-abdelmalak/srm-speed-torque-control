%% SRM Milestone 4 - Academic Plotting Script
% This script extracts data from the Simulink 'out' object and generates
% publication-quality figures with forced white backgrounds.

% Clear previous figures
close all;

%% 1. Speed Tracking Performance
% Extract data
t_w    = out.SpeedData.time;
w_ref  = out.SpeedData.signals(1).values(:, 1);
w_meas = out.SpeedData.signals(1).values(:, 2);

% Create Figure
fig1 = figure('Color', 'w', 'Position', [100, 100, 800, 350]);
plot(t_w, w_ref, '--k', 'LineWidth', 2);
hold on;
plot(t_w, w_meas, 'b', 'LineWidth', 2);
hold off;

% Academic Formatting
grid on;
box on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontSize', 12, 'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Rotor Speed (rad/s)', 'FontSize', 14, 'FontWeight', 'bold');
title('Test 1: Speed Step Response', 'FontSize', 16);
legend('Reference Speed (\omega^*)', 'Measured Speed (\omega)', 'Location', 'best');

%% 2. Phase Currents (Commutation Verification)
% Extract data
t_i = out.CurrentData.time;
i1  = out.CurrentData.signals(1).values(:, 1);
i2  = out.CurrentData.signals(1).values(:, 2);
i3  = out.CurrentData.signals(1).values(:, 3);

% Create Figure
fig2 = figure('Color', 'w', 'Position', [150, 150, 800, 350]);
plot(t_i, i1, 'r', 'LineWidth', 1.5);
hold on;
plot(t_i, i2, 'b', 'LineWidth', 1.5);
plot(t_i, i3, 'color','#006400', 'LineWidth', 1.5); % Dark Green for better contrast
hold off;

% Academic Formatting
grid on;
box on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontSize', 12, 'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Phase Current (A)', 'FontSize', 14, 'FontWeight', 'bold');
title('Phase Current Commutation (Hysteresis Control)', 'FontSize', 16);
legend('Phase 1 (i_1)', 'Phase 2 (i_2)', 'Phase 3 (i_3)', 'Location', 'best');

% Optional: Zoom in to see the switching frequency clearly
% xlim([0.05, 0.1]); 

%% 3. Electromagnetic Torque Dynamics
% Extract data
t_te = out.TorqueData.time;
Te   = out.TorqueData.signals(1).values(:, 1);
TL   = out.TorqueData.signals(1).values(:, 2);

% Create Figure
fig3 = figure('Color', 'w', 'Position', [200, 200, 800, 350]);
plot(t_te, Te, 'b', 'LineWidth', 1.2);
hold on;
plot(t_te, TL, '--r', 'LineWidth', 2);
hold off;

% Academic Formatting
grid on;
box on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontSize', 12, 'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Torque (N.m)', 'FontSize', 14, 'FontWeight', 'bold');
title('Electromagnetic Torque vs Load Torque', 'FontSize', 16);
legend('Electromagnetic Torque (T_e)', 'Load Torque (T_L)', 'Location', 'best');

%% Export for Report (Uncomment to use)
%The following commands will save perfect, high-resolution PNGs to your folder
% exportgraphics(fig1, 'Speed_Response.png', 'Resolution', 300);
% exportgraphics(fig2, 'Phase_Currents.png', 'Resolution', 300);
% exportgraphics(fig3, 'Torque_Ripple.png', 'Resolution', 300);