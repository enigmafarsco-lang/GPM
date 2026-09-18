function cfg = config_mode_A_uav_shallow()
%% Mode A: UAV Shallow GPR — matches published UAV-GPR literature
% Reference: García-Fernández et al., "Airborne Multi-Channel GPR", 2018

cfg = struct();
cfg.mode = 'A_UAV_Shallow';
cfg.description = 'UAV-mounted SFCW GPR for shallow subsurface (buried objects, landmines, pipes)';

% Physics constants
cfg.c = 299792458;
cfg.mu0 = 4*pi*1e-7;
cfg.eps0 = 8.854187817e-12;

% Frequency: high band for shallow high-resolution
cfg.f_start = 500e6;    % 500 MHz
cfg.f_stop  = 3000e6;   % 3 GHz
cfg.n_tones = 256;
cfg.bw = cfg.f_stop - cfg.f_start;
cfg.delta_f = cfg.bw / (cfg.n_tones - 1);
cfg.f_center = (cfg.f_start + cfg.f_stop) / 2;

% UAV parameters — REALISTIC for GPR
cfg.uav_altitude = 0.5;    % 0.5 m above ground (published range: 0.3-1.0 m)
cfg.uav_speed = 0.5;       % Slow flight for stability
cfg.scan_length = 10;
cfg.n_positions = 100;
cfg.n_averages = 32;

% Antenna
cfg.antenna_type = 'Vivaldi';
cfg.tx_antenna_gain_dBi = 8;
cfg.rx_antenna_gain_dBi = 8;
cfg.tx_rx_separation = 0.2;   % m
cfg.antenna_coupling_dB = -40; % Isolation between TX and RX antennas

% Depth targets
cfg.target_depths = [0.10, 0.25, 0.50, 0.80];  % 10cm - 80cm
cfg.target_x      = [2.0,  4.0,  6.0,  8.0];
cfg.target_rcs    = [0.01, 0.05, 0.2, 0.5];    % m² (small buried objects)
cfg.n_targets = length(cfg.target_depths);

% Processing depth window
cfg.depth_min = 0.05;
cfg.depth_max = 1.0;

% Soil (typical field soil, dry)
cfg.soil_moisture = 0.08;
cfg.soil_conductivity = 0.005;   % S/m (dry sandy soil)
cfg.soil_eps_r_est = 5;          % Approximate for feasibility check

% RF chain (typical SDR + amplifier)
cfg.dac_power_dBm = 0;
cfg.pa_gain_dB = 30;             % Modest PA (10 W)
cfg.tx_cable_loss_dB = 1;
cfg.rx_cable_loss_dB = 1;
cfg.lna_gain_dB = 20;
cfg.system_nf_dB = 3;
cfg.adc_enob = 11;

% Processing
cfg.n_ifft = 2048;
cfg.kaiser_beta = 6;
cfg.cfar_pfa = 1e-4;

% Derived
cfg.eirp_dBm = cfg.dac_power_dBm + cfg.pa_gain_dB - ...
    cfg.tx_cable_loss_dB + cfg.tx_antenna_gain_dBi;

end
