function cfg = config_mode_A_uav_shallow()
%% Mode A: UAV-mounted shallow SFCW GPR (buried objects, landmines, pipes)
%
% Reference: Garcia-Fernandez et al., "Airborne Multi-Channel GPR", 2018;
% typical 0.5-3 GHz UAV GPR bands with 0.3-1.0 m standoff.
%
% Everything the pipeline needs is in this struct; gpr_check_config validates
% it and adds the derived fields (soil permittivity, bin sizes, EIRP, ...).

cfg = struct();
cfg.mode = 'A_UAV_Shallow';
cfg.description = 'UAV-mounted SFCW GPR for the shallow subsurface';
cfg.reference = 'Garcia-Fernandez et al., Airborne Multi-Channel GPR, 2018';

% ------------------------------------------------------------- frequency plan
cfg.f_start = 500e6;       % Hz
cfg.f_stop  = 3000e6;      % Hz
cfg.n_tones = 256;         % frequency steps
cfg.tone_dwell_time = 50e-6;   % s per tone -> 20 kHz noise bandwidth

% ------------------------------------------------------------------- platform
cfg.uav_altitude = 0.5;    % m above ground (published range 0.3-1.0 m)
cfg.uav_speed    = 0.5;    % m/s, slow flight for phase stability
cfg.scan_length  = 10;     % m of surveyed profile
cfg.n_positions  = 200;    % traces along the profile (0.05 m spacing).
                           % 0.1 m spacing is too coarse to migrate at 0.1 m
                           % depth: the first Fresnel zone there is only 7 cm,
                           % so a coarser grid leaves the diffraction tail of
                           % every target unfocused (see gpr_check_config).
cfg.n_averages   = 32;     % coherent sweeps per trace

% -------------------------------------------------------------------- antenna
cfg.antenna_type        = 'Vivaldi';
cfg.tx_antenna_gain_dBi = 8;
cfg.rx_antenna_gain_dBi = 8;
cfg.tx_rx_separation    = 0.2;    % m
cfg.antenna_coupling_dB = -40;    % TX->RX isolation
cfg.beam_half_angle_tan = tan(pi/6);   % 60 deg total beamwidth
cfg.footprint_min       = 0.30;   % m, near-field floor of the footprint

% -------------------------------------------------------------------- targets
cfg.target_depths = [0.10, 0.25, 0.50, 0.80];   % m
cfg.target_x      = [2.0,  4.0,  6.0,  8.0];    % m along the profile
cfg.target_rcs    = [0.01, 0.05, 0.20, 0.50];   % m^2 (small buried objects)

% --------------------------------------------------------- processing window
cfg.depth_min = 0.05;      % m, first depth bin examined by the CFAR
cfg.depth_max = 1.00;      % m, last depth bin examined / imaged

% ----------------------------------------------------------------------- soil
cfg.soil_moisture     = 0.08;    % m^3/m^3 (dry field soil)
cfg.soil_conductivity = 0.005;   % S/m (dry sand)

% ------------------------------------------------------------------- RF chain
cfg.dac_power_dBm     = 0;       % dBm into the predriver
cfg.pa_gain_dB        = 30;      % dB
cfg.pa_p1dB_dBm       = 35;      % dBm, PA compression point (~3 W class)
cfg.pa_backoff_dB     = 3;       % dB above P1dB the drive is set
cfg.tx_cable_loss_dB  = 1;
cfg.rx_cable_loss_dB  = 1;
cfg.rx_cable_delay    = 5e-9;    % s, removed by B09_Calibration
cfg.lna_gain_dB       = 20;
cfg.system_nf_dB      = 3;
% 14 bits: the surface bounce sits ~50 dB above the deepest target, and a
% 6.02*ENOB converter must cover that plus a CFAR margin.
cfg.adc_enob          = 14;

% --------------------------------------------------------------- processing
cfg.n_ifft             = 2048;   % zero-padding factor 8 over n_tones
cfg.kaiser_beta        = 8;   % -58 dB range sidelobes, 1.79x mainlobe
cfg.cfar_pfa           = 1e-6;
cfg.cfar_estimator     = 'mean';  % classical CA-CFAR: this scene is sparse, so the
                                 % training window is almost pure noise and the
                                 % arithmetic mean is the best (lowest variance)
                                 % estimator.  'log' would sit ~10x lower here and
                                 % flag the pulse-compression skirt of every target.
cfg.cfar_guard         = 8;      % cells, must cover the compressed pulse
cfg.cfar_train         = 16;     % cells of training ring per side
cfg.bg_remove_modes    = 1;      % component-wise median across positions (robust to targets)
cfg.range_window       = 'kaiser'; % taper the spectrum (see gpr_range_window) for sidelobe
                                 % control; the beta below trades mainlobe width vs. sidelobes
cfg.migration_aperture = 1.0;    % aperture = 1.0 * depth (matches the antenna illumination)
cfg.tx_taper_percent   = 0;      % 0 = flat comb (B10 applies the Kaiser)

cfg = gpr_check_config(cfg, 'Mode A');
end
