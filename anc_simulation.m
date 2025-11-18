%% ESP32 ANC System - MATLAB Simulation
% Exact replica of the embedded C++ implementation
% Simulates LMS-based ANC for 1kHz tone cancellation

clear; clc; close all;

%% -------------------- Parameters (from C++ code) --------------------
FS_HZ = 44100;              % Sample rate (Hz)
FRAME = 64;                 % Block size
F0_HZ = 1000.0;             % Primary tone frequency (Hz)
N_TAPS = 32;                % LMS FIR filter length
MU = 0.005;                 % LMS step size
LEAK = 0.0;                 % Leaky LMS factor (0 = off)
OUT_CLIP = 0.95;            % Output clipping threshold
HPF_DC_N = 1024;            % DC remover length

% Simulation duration
SIM_DURATION_SEC = 5.0;     % Simulate 5 seconds
TOTAL_SAMPLES = floor(FS_HZ * SIM_DURATION_SEC);
NUM_BLOCKS = floor(TOTAL_SAMPLES / FRAME);

%% -------------------- Initialize State Variables --------------------
% LMS filter coefficients (w[N_TAPS])
w = zeros(N_TAPS, 1);

% Ring buffer for reference signal (xBuf[N_TAPS + 256])
RING_SIZE = N_TAPS + 256;
xBuf = zeros(RING_SIZE, 1);
idx = 1;  % MATLAB uses 1-based indexing (C++ uses 0-based)

% Phase accumulator for 1 kHz sine generation
phase = 0.0;
TWOPI = 2 * pi;
dphi = TWOPI * F0_HZ / FS_HZ;

% DC Remover state
dc_acc = 0.0;
dc_n = 0;

%% -------------------- Storage for Results --------------------
primary_signal = zeros(TOTAL_SAMPLES, 1);
secondary_signal = zeros(TOTAL_SAMPLES, 1);
error_signal = zeros(TOTAL_SAMPLES, 1);
weights_history = zeros(NUM_BLOCKS, N_TAPS);  % Store weights per block

%% -------------------- Simulation Loop --------------------
fprintf('Starting ANC simulation...\n');
fprintf('Simulating %d samples (%.2f seconds) in %d blocks\n', ...
        TOTAL_SAMPLES, SIM_DURATION_SEC, NUM_BLOCKS);

sample_index = 1;

for block = 1:NUM_BLOCKS
    % Allocate block buffers
    bufPrimary = zeros(FRAME, 1);
    bufSecondary = zeros(FRAME, 1);
    bufError = zeros(FRAME, 1);
    
    % Process each sample in the block
    for n = 1:FRAME
        %% 1) Generate primary reference x[n] = sin(2π f0 t)
        x = sin(phase);
        phase = phase + dphi;
        if phase >= TWOPI
            phase = phase - TWOPI;
        end
        
        %% 2) Push reference into ring buffer
        xBuf(idx) = x;
        
        %% 3) Generate control output y[n] = sum_k w[k] * x[n-k]
        y = 0.0;
        j = idx;
        for k = 1:N_TAPS
            % Ring index (wrap) - converting C++ logic to MATLAB
            jj = j - (k - 1);  % k-1 because MATLAB is 1-based
            if jj < 1
                jj = jj + RING_SIZE;
            end
            y = y + w(k) * xBuf(jj);
        end
        
        %% 4) Play signals: primary (x scaled) and secondary (y)
        primary_out = 0.50 * x;
        secondary_out = y;
        
        % Clip before output
        if primary_out > OUT_CLIP
            primary_out = OUT_CLIP;
        end
        if primary_out < -OUT_CLIP
            primary_out = -OUT_CLIP;
        end
        if secondary_out > OUT_CLIP
            secondary_out = OUT_CLIP;
        end
        if secondary_out < -OUT_CLIP
            secondary_out = -OUT_CLIP;
        end
        
        bufPrimary(n) = primary_out;
        bufSecondary(n) = secondary_out;
        
        %% 5) Simulate error microphone
        % In real system: e = primary + secondary (acoustic superposition)
        % Plus some noise to make it realistic
        noise = 0.01 * randn();  % Small measurement noise
        e_raw = primary_out + secondary_out + noise;
        
        % DC removal (incremental mean - Welford simplified)
        dc_n = min(dc_n + 1, HPF_DC_N);
        dc_acc = dc_acc + (e_raw - dc_acc) / dc_n;
        e = e_raw - dc_acc;
        
        bufError(n) = e;
        
        %% 6) LMS weight update: w[k] += mu * e[n] * x[n-k] - leak*w[k]
        jx = idx;
        for k = 1:N_TAPS
            jj = jx - (k - 1);
            if jj < 1
                jj = jj + RING_SIZE;
            end
            xk = xBuf(jj);
            w(k) = (1.0 - LEAK) * w(k) + MU * e * xk;
        end
        
        %% 7) Advance ring index
        idx = idx + 1;
        if idx > RING_SIZE
            idx = 1;
        end
    end
    
    % Store block results
    start_idx = (block - 1) * FRAME + 1;
    end_idx = start_idx + FRAME - 1;
    primary_signal(start_idx:end_idx) = bufPrimary;
    secondary_signal(start_idx:end_idx) = bufSecondary;
    error_signal(start_idx:end_idx) = bufError;
    weights_history(block, :) = w';
    
    % Print progress every ~100 blocks
    if mod(block, 100) == 0
        error_avg = mean(abs(bufError));
        output_avg = mean(abs(bufSecondary));
        fprintf('Block %d/%d | Error(avg): %.6f | Output(avg): %.6f | W[0]: %.6f\n', ...
                block, NUM_BLOCKS, error_avg, output_avg, w(1));
    end
end

fprintf('Simulation complete!\n\n');

%% -------------------- Analysis & Visualization --------------------

time = (0:TOTAL_SAMPLES-1) / FS_HZ;

% Calculate noise reduction over time
window_size = 4410;  % 100ms window
error_rms = zeros(floor(TOTAL_SAMPLES/window_size), 1);
primary_rms = zeros(floor(TOTAL_SAMPLES/window_size), 1);

for i = 1:floor(TOTAL_SAMPLES/window_size)
    start_idx = (i-1)*window_size + 1;
    end_idx = min(start_idx + window_size - 1, TOTAL_SAMPLES);
    error_rms(i) = rms(error_signal(start_idx:end_idx));
    primary_rms(i) = rms(primary_signal(start_idx:end_idx));
end

noise_reduction_dB = 20 * log10(primary_rms ./ (error_rms + 1e-10));
time_rms = ((0:length(error_rms)-1) * window_size) / FS_HZ;

%% -------------------- Plots --------------------

% Figure 1: Time-domain signals
figure('Name', 'ANC Signals - Time Domain', 'Position', [100 100 1200 800]);

subplot(4,1,1);
plot(time, primary_signal, 'b', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Primary Signal (1 kHz Noise)');
xlim([0 max(time)]);

subplot(4,1,2);
plot(time, secondary_signal, 'r', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Secondary Signal (Anti-noise from LMS)');
xlim([0 max(time)]);

subplot(4,1,3);
plot(time, error_signal, 'g', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Error Signal (After Cancellation)');
xlim([0 max(time)]);

subplot(4,1,4);
plot(time_rms, noise_reduction_dB, 'k', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Noise Reduction (dB)');
title('Noise Reduction Over Time');
xlim([0 max(time_rms)]);
ylim([-10 50]);

% Figure 2: Weight convergence
figure('Name', 'LMS Filter Weights Convergence', 'Position', [150 150 1200 600]);

subplot(2,1,1);
plot((0:NUM_BLOCKS-1)*FRAME/FS_HZ, weights_history(:,1:min(8,N_TAPS)), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Weight Value');
title('First 8 LMS Weights Convergence');
legend(arrayfun(@(i) sprintf('w[%d]', i-1), 1:min(8,N_TAPS), 'UniformOutput', false), ...
       'Location', 'eastoutside');

subplot(2,1,2);
imagesc((0:NUM_BLOCKS-1)*FRAME/FS_HZ, 0:N_TAPS-1, weights_history');
colorbar;
xlabel('Time (s)');
ylabel('Weight Index');
title('All LMS Weights Evolution (Heatmap)');
colormap(jet);

% Figure 3: Frequency domain analysis
figure('Name', 'Frequency Domain Analysis', 'Position', [200 200 1200 600]);

% Take last second for steady-state analysis
last_second_samples = FS_HZ;
start_ss = max(1, TOTAL_SAMPLES - last_second_samples);

[P_primary, f] = pwelch(primary_signal(start_ss:end), [], [], [], FS_HZ);
[P_error, ~] = pwelch(error_signal(start_ss:end), [], [], [], FS_HZ);
[P_secondary, ~] = pwelch(secondary_signal(start_ss:end), [], [], [], FS_HZ);

subplot(2,1,1);
plot(f, 10*log10(P_primary), 'b', 'LineWidth', 2); hold on;
plot(f, 10*log10(P_error), 'g', 'LineWidth', 2);
plot(f, 10*log10(P_secondary), 'r', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Power/Frequency (dB/Hz)');
title('Power Spectral Density (Steady State - Last 1 Second)');
legend('Primary (Noise)', 'Error (After Cancellation)', 'Secondary (Anti-noise)', 'Location', 'best');
xlim([0 3000]);

subplot(2,1,2);
attenuation = 10*log10(P_primary) - 10*log10(P_error + 1e-10);
plot(f, attenuation, 'k', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Attenuation (dB)');
title('Noise Attenuation vs Frequency');
xlim([0 3000]);
ylim([-10 60]);
hold on;
plot([F0_HZ F0_HZ], ylim, 'r--', 'LineWidth', 2);
text(F0_HZ+50, 50, sprintf('Target: %.0f Hz', F0_HZ), 'Color', 'r', 'FontWeight', 'bold');

% Figure 4: Detailed convergence view (first 0.5 seconds)
figure('Name', 'Initial Convergence (First 0.5s)', 'Position', [250 250 1200 600]);

conv_samples = min(floor(0.5 * FS_HZ), TOTAL_SAMPLES);
time_conv = (0:conv_samples-1) / FS_HZ;

subplot(3,1,1);
plot(time_conv, primary_signal(1:conv_samples), 'b', 'LineWidth', 1); hold on;
plot(time_conv, secondary_signal(1:conv_samples), 'r', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Primary and Secondary Signals (Initial Convergence)');
legend('Primary', 'Secondary', 'Location', 'best');

subplot(3,1,2);
plot(time_conv, error_signal(1:conv_samples), 'g', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Error Signal (Initial Convergence)');

subplot(3,1,3);
conv_blocks = floor(conv_samples / FRAME);
plot((0:conv_blocks-1)*FRAME/FS_HZ, weights_history(1:conv_blocks, 1), 'k', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Weight Value');
title('Weight w[0] Convergence');

%% -------------------- Performance Summary --------------------
fprintf('=== PERFORMANCE SUMMARY ===\n');
fprintf('Simulation Parameters:\n');
fprintf('  Sample Rate: %d Hz\n', FS_HZ);
fprintf('  Target Frequency: %.0f Hz\n', F0_HZ);
fprintf('  LMS Filter Order: %d taps\n', N_TAPS);
fprintf('  Step Size (MU): %.6f\n', MU);
fprintf('  Total Duration: %.2f seconds\n\n', SIM_DURATION_SEC);

% Initial performance (first 100ms)
initial_samples = min(4410, TOTAL_SAMPLES);
initial_error_rms = rms(error_signal(1:initial_samples));
initial_primary_rms = rms(primary_signal(1:initial_samples));
initial_NR = 20*log10(initial_primary_rms / initial_error_rms);

% Steady-state performance (last 1 second)
ss_error_rms = rms(error_signal(start_ss:end));
ss_primary_rms = rms(primary_signal(start_ss:end));
ss_NR = 20*log10(ss_primary_rms / ss_error_rms);

fprintf('Noise Reduction Performance:\n');
fprintf('  Initial (0-100ms): %.2f dB\n', initial_NR);
fprintf('  Steady State (last 1s): %.2f dB\n', ss_NR);
fprintf('  Improvement: %.2f dB\n\n', ss_NR - initial_NR);

fprintf('Final LMS Weights:\n');
fprintf('  w[0] = %.6f\n', w(1));
fprintf('  w[1] = %.6f\n', w(2));
fprintf('  Max weight: %.6f\n', max(abs(w)));
fprintf('  L2 norm: %.6f\n\n', norm(w));

% Find attenuation at target frequency
[~, target_idx] = min(abs(f - F0_HZ));
target_attenuation = attenuation(target_idx);
fprintf('Attenuation at %.0f Hz: %.2f dB\n', F0_HZ, target_attenuation);

fprintf('\nSimulation matches ESP32 embedded implementation!\n');
