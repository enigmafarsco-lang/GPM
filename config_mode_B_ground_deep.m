function cfg = config_mode_B_ground_deep()
%% Mode B: ground-coupled low-frequency SFCW GPR for deep detection (3-15 m)
%
% Reference: Annan, "Ground Penetrating Radar Principles", 2005; typical
% 10-100 MHz loaded-bowtie systems for deep utility/geological surveying.

cfg = struct();
cfg.mode = 'B_Ground_Deep';
cfg.description = 'Ground-coupled low-frequency SFCW GPR for deep detection';
cfg.reference = 'Annan, Ground Penetrating Radar Principles, 2005';

% ------------------------------------------------------------- frequency plan
cfg.f_start = 10e6;        % Hz
cfg.f_stop  = 100e6;       % Hz
cfg.n_tones = 512;         % fine tone spacing -> large unambiguous range
cfg.tone_dwell_time = 200e-6;  % s per tone -> 5 kHz noise bandwidth

% ------------------------------------------------------------------- platform
cfg.uav_altitude = 0.02;   % m ground clearance of the antenna skid
cfg.uav_speed    = 0.1;    % m/s walking pace
cfg.scan_length  = 20;     % m of surveyed profile
cfg.n_positions  = 200;    % traces (0.1 m spacing)
cfg.n_averages   = 128;    % coherent sweeps per trace

% -------------------------------------------------------------------- antenna
cfg.antenna_type        = 'Loaded_Bowtie';
cfg.tx_antenna_gain_dBi = 2;
cfg.rx_antenna_gain_dBi = 2;
cfg.tx_rx_separation    = 1.5;    % m
cfg.antenna_coupling_dB = -30;    % TX->RX isolation
cfg.beam_half_angle_tan = tan(pi/6);
cfg.footprint_min       = 0.30;

% -------------------------------------------------------------------- targets
cfg.target_depths = [3.0, 5.0, 8.0, 12.0, 15.0];   % m
cfg.target_x      = [4.0, 8.0, 12.0, 16.0, 18.0];  % m
cfg.target_rcs    = [2.0, 5.0, 10.0, 20.0, 30.0];  % m^2

% --------------------------------------------------------- processing window
cfg.depth_min = 2.0;       % m
cfg.depth_max = 18.0;      % m

% ----------------------------------------------------------------------- soil
cfg.soil_moisture     = 0.05;    % m^3/m^3 (dry sand/loam: required for depth)
cfg.soil_conductivity = 0.001;   % S/m

% ------------------------------------------------------------------- RF chain
cfg.dac_power_dBm     = 0;
cfg.pa_gain_dB        = 47;      % ~50 W PA
cfg.pa_p1dB_dBm       = 50;      % dBm (100 W class)
cfg.pa_backoff_dB     = 3;
cfg.tx_cable_loss_dB  = 2;
cfg.rx_cable_loss_dB  = 2;
cfg.rx_cable_delay    = 12e-9;   % longer cable run on a cart system
cfg.lna_gain_dB       = 25;
cfg.system_nf_dB      = 3;
% 16 bits: deep GPR needs the extra dynamic range because the direct
% coupling and the surface bounce are ~70 dB above a 15 m target.
cfg.adc_enob          = 16;

% --------------------------------------------------------------- processing
cfg.n_ifft             = 4096;
cfg.kaiser_beta        = 8;
cfg.cfar_pfa           = 1e-5;
cfg.cfar_estimator     = 'log';  % robust geometric-mean CFAR: five targets share a
                                 % 20 m section, so every training window contains a
                                 % brighter neighbour.  The arithmetic mean reports
                                 % that neighbour as noise (measured: 1e11 x the true
                                 % floor next to the 15 m target) and hides 3 of the
                                 % 5 targets; the log-domain level does not move.
cfg.cfar_guard         = 8;
cfg.cfar_train         = 16;
cfg.bg_remove_modes    = 1;      % component-wise median across positions (robust to targets)
cfg.range_window       = 'kaiser'; % sidelobe control: without it the shallow targets' range
                                 % sidelobes sum coherently in the migration and mask T5
cfg.migration_aperture = 1.0;    % aperture = 1.0 * depth (matches the antenna illumination)
cfg.tx_taper_percent   = 0;

cfg = gpr_check_config(cfg, 'Mode B');
end
