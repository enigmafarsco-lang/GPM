function cfg = config_mode_B_ground_deep()
%% Mode B: Ground-Coupled Deep GPR — realistic for 3-15 m depth
% Reference: Annan (2005) Ground Penetrating Radar Principles

cfg = struct();
cfg.mode = 'B_Ground_Deep';
cfg.description = 'Ground-coupled low-frequency SFCW GPR for deep detection';

cfg.c = 299792458;
cfg.mu0 = 4*pi*1e-7;
cfg.eps0 = 8.854187817e-12;

% Frequency: LOW BAND for penetration
cfg.f_start = 10e6;      % 10 MHz
cfg.f_stop  = 100e6;     % 100 MHz
cfg.n_tones = 512;       % Fine tone spacing for large unambiguous range
cfg.bw = cfg.f_stop - cfg.f_start;
cfg.delta_f = cfg.bw / (cfg.n_tones - 1);
cfg.f_center = (cfg.f_start + cfg.f_stop) / 2;

% Ground-coupled: no UAV, antenna dragged along surface
cfg.uav_altitude = 0.02;   % 2 cm ground clearance
cfg.uav_speed = 0.1;       % 10 cm/s (walking pace)
cfg.scan_length = 20;
cfg.n_positions = 200;
cfg.n_averages = 128;

% Antenna: large loaded bowtie or resistively loaded dipole
cfg.antenna_type = 'Loaded_Bowtie';
cfg.tx_antenna_gain_dBi = 2;
cfg.rx_antenna_gain_dBi = 2;
cfg.tx_rx_separation = 1.5;      % m (larger to reduce coupling)
cfg.antenna_coupling_dB = -30;

% Target depths — realistic deep detection
cfg.target_depths = [3.0, 5.0, 8.0, 12.0, 15.0];
cfg.target_x      = [4.0, 8.0, 12.0, 16.0, 18.0];
cfg.target_rcs    = [2.0, 5.0, 10.0, 20.0, 30.0];  % Larger targets required
cfg.n_targets = length(cfg.target_depths);

% Processing window — matches target depths
cfg.depth_min = 2.0;
cfg.depth_max = 18.0;

% Soil (favorable dry conditions required for deep GPR)
cfg.soil_moisture = 0.05;
cfg.soil_conductivity = 0.001;   % S/m (dry sandy loam, ideal for deep GPR)
cfg.soil_eps_r_est = 4;

% High-power RF chain
cfg.dac_power_dBm = 0;
cfg.pa_gain_dB = 47;             % ~50 W PA
cfg.tx_cable_loss_dB = 2;
cfg.rx_cable_loss_dB = 2;
cfg.lna_gain_dB = 25;
cfg.system_nf_dB = 3;
cfg.adc_enob = 12;

% Processing
cfg.n_ifft = 4096;
cfg.kaiser_beta = 8;
cfg.cfar_pfa = 1e-5;

cfg.eirp_dBm = cfg.dac_power_dBm + cfg.pa_gain_dB - ...
    cfg.tx_cable_loss_dB + cfg.tx_antenna_gain_dBi;

end
