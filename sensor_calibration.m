%% Water Quality Sensor Calibration and Visualization
% Authors: Khalid Hossain, Soumik Saha
% Description: Calibration curves for pH and turbidity sensors used in
%              IoT-based fish farm water monitoring system

clear; clc; close all;

%% ========== pH Sensor Calibration ==========

% Calibration data points (voltage, pH)
voltages = [3.11, 2.62, 2.25];
pH_values = [4.00, 6.86, 9.18];

% Voltage range for plotting (1V to 4V)
v = linspace(1.0, 4.0, 1000);

% Calibration constants
V_THRESHOLD = 2.62;  % Voltage at pH 6.86
V_BUFFER = 0.02;     % ±0.02V deadband

% Separate voltage regions
v_high = v(v > (V_THRESHOLD + V_BUFFER));
v_buffer = v(v >= (V_THRESHOLD - V_BUFFER) & v <= (V_THRESHOLD + V_BUFFER));
v_low = v(v < (V_THRESHOLD - V_BUFFER));

% Calculate pH for each region
pH_high = -5.84 * v_high + 22.16;  % High voltage region (acidic pH)
pH_low = -6.27 * v_low + 23.28;     % Low voltage region (basic pH)

% Interpolated buffer zone for smooth transition
pH_buffer = zeros(size(v_buffer));
for i = 1:length(v_buffer)
    pH1 = -5.84 * v_buffer(i) + 22.16;
    pH2 = -6.27 * v_buffer(i) + 23.28;
    weight = (v_buffer(i) - (V_THRESHOLD - V_BUFFER)) / (2 * V_BUFFER);
    pH_buffer(i) = pH1 * (1 - weight) + pH2 * weight;
end

% Plot pH calibration curve
figure('Name', 'pH Sensor Calibration');
hold on;

% Plot different regions with distinct colors
plot(v_high, pH_high, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Acidic Range');
plot(v_buffer, pH_buffer, 'g-', 'LineWidth', 2.5, 'DisplayName', 'Buffer Zone');
plot(v_low, pH_low, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Basic Range');

% Mark calibration points
plot(voltages, pH_values, 'ko', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'k', 'DisplayName', 'Calibration Points');

% Annotate calibration points
labels = {'pH 4.0 @ 3.11V', 'pH 6.86 @ 2.62V', 'pH 9.18 @ 2.25V'};
for i = 1:length(voltages)
    text(voltages(i) + 0.05, pH_values(i), labels{i}, ...
         'VerticalAlignment', 'bottom', 'FontSize', 11, 'FontWeight', 'bold');
end

% Formatting
xlabel('Sensor Voltage (V)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('pH Value', 'FontSize', 14, 'FontWeight', 'bold');
title('pH Sensor Calibration Curve with Piecewise Linear Fit', ...
      'FontSize', 16, 'FontWeight', 'bold');
grid on;
legend('Location', 'northeast', 'FontSize', 12);
xlim([1.0, 4.0]);
ylim([3.5, 10.0]);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

%% ========== Turbidity Sensor Calibration ==========

% Calibration endpoints
analog_clean = 800;  % Clean water (0% turbidity)
analog_dirty = 0;    % Dirty water (100% turbidity)

% Generate analog values (dirty to clean)
analog_values = linspace(analog_dirty, analog_clean, 500);

% Linear mapping: higher ADC = cleaner water = lower turbidity %
turbidity_percent = 100 * (analog_clean - analog_values) / ...
                    (analog_clean - analog_dirty);

% Alert threshold
threshold = 30;  % 30% turbidity threshold
analog_threshold = analog_clean - (threshold / 100) * (analog_clean - analog_dirty);

% Plot turbidity calibration curve
figure('Name', 'Turbidity Sensor Calibration');
plot(analog_values, turbidity_percent, 'b-', 'LineWidth', 2.5);
hold on;

% Mark threshold lines
yline(threshold, 'r--', 'LineWidth', 2, 'DisplayName', '30% Alert Threshold');
xline(analog_threshold, 'r--', 'LineWidth', 2, 'HandleVisibility', 'off');

% Mark calibration endpoints
plot([analog_dirty, analog_clean], [100, 0], 'ko', ...
     'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Calibration Points');

% Annotations
text(analog_threshold + 30, threshold + 5, ...
     sprintf('Alert at %d%% (ADC = %d)', threshold, round(analog_threshold)), ...
     'Color', 'red', 'FontSize', 11, 'FontWeight', 'bold');

% Formatting
xlabel('Analog Sensor Value (ADC)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Turbidity (%)', 'FontSize', 14, 'FontWeight', 'bold');
title('Turbidity Sensor Calibration Curve', 'FontSize', 16, 'FontWeight', 'bold');
grid on;
legend('Location', 'SouthWest', 'FontSize', 12);
xlim([analog_dirty, analog_clean]);
ylim([0, 105]);
set(gca, 'FontSize', 12, 'LineWidth', 1.5);

%% ========== Save Figures ==========
% Uncomment to save as high-resolution images
% saveas(gcf(1), 'pH_calibration.png');
% saveas(gcf(2), 'turbidity_calibration.png');

fprintf('Calibration plots generated successfully.\n');
fprintf('pH range: %.1f - %.1f (Voltage: %.2fV - %.2fV)\n', ...
        min(pH_values), max(pH_values), min(voltages), max(voltages));
fprintf('Turbidity range: 0%% - 100%% (ADC: %d - %d)\n', ...
        analog_dirty, analog_clean);
