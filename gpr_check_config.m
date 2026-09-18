function cfg = gpr_check_config(cfg, name)
% GPR_CHECK_CONFIG  Validate a configuration struct and add derived fields.
%
%   cfg = GPR_CHECK_CONFIG(cfg) errors out on anything the pipeline cannot
%   handle (mismatched target vectors, n_ifft < n_tones, non-physical soil
%   values, ...) and fills in the derived fields that the rest of the package
%   needs. Called by both configuration files, so a hand-edited config is
%   checked exactly like a shipped one.
%
%   cfg = GPR_CHECK_CONFIG(cfg, 'Mode A') uses NAME in the error identifiers.

if nargin < 2 || isempty(name)
    name = 'cfg';
end
bad = @(msg, varargin) error('gpr:config:invalid', sprintf(['%s: ' msg], name, varargin{:}));

req = {'mode', 'f_start', 'f_stop', 'n_tones', 'uav_altitude', 'scan_length', ...
    'n_positions', 'n_averages', 'target_depths', 'target_x', 'target_rcs', ...
    'depth_min', 'depth_max', 'soil_moisture', 'soil_conductivity', ...
    'pa_gain_dB', 'lna_gain_dB', 'system_nf_dB', 'adc_enob', 'n_ifft', ...
    'kaiser_beta', 'cfar_pfa', 'tx_rx_separation', 'antenna_coupling_dB', ...
    'tx_antenna_gain_dBi', 'rx_antenna_gain_dBi', 'dac_power_dBm', ...
    'tx_cable_loss_dB', 'rx_cable_loss_dB'};
for k = 1:numel(req)
    if ~isfield(cfg, req{k})
        bad('missing field ''%s''.', req{k});
    end
end

if ~ischar(cfg.mode) && ~isstring(cfg.mode)
    bad('mode must be text.');
end
cfg.mode = char(cfg.mode);

if ~(cfg.f_start > 0) || ~(cfg.f_stop > cfg.f_start)
    bad('need 0 < f_start < f_stop (got %.4g and %.4g).', cfg.f_start, cfg.f_stop);
end
if cfg.n_tones < 2 || cfg.n_tones ~= round(cfg.n_tones)
    bad('n_tones must be an integer >= 2 (got %g).', cfg.n_tones);
end
if cfg.n_positions < 2 || cfg.n_positions ~= round(cfg.n_positions)
    bad('n_positions must be an integer >= 2 (got %g).', cfg.n_positions);
end
if cfg.n_ifft < cfg.n_tones
    bad('n_ifft (%d) must be >= n_tones (%d).', cfg.n_ifft, cfg.n_tones);
end
if mod(log2(cfg.n_ifft), 1) ~= 0
    bad('n_ifft (%d) must be a power of two (the range axis is a radix-2 IFFT).', cfg.n_ifft);
end
if cfg.n_averages < 1
    bad('n_averages must be >= 1 (got %g).', cfg.n_averages);
end
if ~(cfg.cfar_pfa > 0) || ~(cfg.cfar_pfa < 1)
    bad('cfar_pfa must be in (0,1) (got %g).', cfg.cfar_pfa);
end
if ~(cfg.depth_max > cfg.depth_min) || ~(cfg.depth_min >= 0)
    bad('need 0 <= depth_min < depth_max (got %g and %g).', cfg.depth_min, cfg.depth_max);
end
if ~(cfg.soil_moisture >= 0) || ~(cfg.soil_moisture < 0.6)
    bad('soil_moisture must be in [0, 0.6) m^3/m^3 (got %g).', cfg.soil_moisture);
end
if ~(cfg.soil_conductivity > 0)
    bad('soil_conductivity must be > 0 S/m (got %g).', cfg.soil_conductivity);
end
if cfg.uav_altitude < 0
    bad('uav_altitude must be >= 0 m (got %g).', cfg.uav_altitude);
end
if cfg.tx_rx_separation < 0
    bad('tx_rx_separation must be >= 0 m (got %g).', cfg.tx_rx_separation);
end
if cfg.adc_enob < 4 || cfg.adc_enob > 24
    bad('adc_enob must be in [4, 24] bits (got %g).', cfg.adc_enob);
end
if cfg.kaiser_beta < 0
    bad('kaiser_beta must be >= 0 (got %g).', cfg.kaiser_beta);
end

td = cfg.target_depths(:).';
tx = cfg.target_x(:).';
tr = cfg.target_rcs(:).';
if numel(td) ~= numel(tx) || numel(td) ~= numel(tr)
    bad('target_depths, target_x and target_rcs must have the same length (%d/%d/%d).', ...
        numel(td), numel(tx), numel(tr));
end
if isempty(td)
    bad('at least one target is required.');
end
if any(td <= 0)
    bad('target depths must be > 0 m.');
end
if any(tr < 0)
    bad('target RCS must be >= 0 m^2.');
end
if any(tx < 0) || any(tx > cfg.scan_length)
    bad('target_x must lie inside the scan (0 .. %g m).', cfg.scan_length);
end
cfg.target_depths = td;
cfg.target_x = tx;
cfg.target_rcs = tr;
cfg.n_targets = numel(td);

% ------------------------------------------------------------- derived fields
cfg.bw       = cfg.f_stop - cfg.f_start;
cfg.delta_f  = cfg.bw/(cfg.n_tones - 1);
cfg.f_center = (cfg.f_start + cfg.f_stop)/2;
cfg.eirp_dBm = cfg.dac_power_dBm + cfg.pa_gain_dB - cfg.tx_cable_loss_dB + cfg.tx_antenna_gain_dBi;

% Soil properties at the centre frequency, from the same model that the
% Simulink blocks use. The previous version hard-coded soil_eps_r_est, which
% disagreed with the model by more than 50% and made every downstream number
% (resolution, unambiguous depth, attenuation, link budget) inconsistent.
[eps_r, ~, alpha, ~, v] = soil_permittivity(cfg.f_center, cfg.soil_moisture, cfg.soil_conductivity);
cfg.soil_eps_r_est = eps_r;
cfg.soil_v         = v;
cfg.soil_alpha_Np  = alpha;
cfg.soil_alpha_dB  = alpha*20/log(10);

% Sampling relations
cfg.dt_bin = 1/(cfg.n_ifft*cfg.delta_f);
cfg.dr_bin = cfg.dt_bin*v/2;

% Range windowing.  An untapered spectrum has -13 dB range sidelobes; after
% migration those sidelobes of a strong shallow target add up coherently and
% raise the CFAR noise estimate around weaker deep targets.  The taper cost
% is a broadened mainlobe, tracked here so every downstream number
% (resolution, CFAR guard, feasibility report) stays consistent.
cfg.range_window = lower(gpr_cfg_value(cfg, 'range_window', 'kaiser'));
if strcmp(cfg.range_window, 'none')
    cfg.range_window = 'rectangular';
end
% Measure the compressed-pulse shape of the actual taper instead of relying
% on a hand-written table: the same window definition is used by block B10.
try
    [cfg.window_broadening, nullw, cfg.cfar_guard_min] = ...
        gpr_pulse_width(cfg.range_window, cfg.n_tones, cfg.kaiser_beta, cfg.n_ifft);
catch err
    bad('%s', err.message);
    cfg.window_broadening = 1; nullw = 2; cfg.cfar_guard_min = 1;
end
cfg.range_resolution  = v/(2*cfg.bw);                  % rectangular limit
cfg.depth_resolution  = cfg.window_broadening*cfg.range_resolution;
cfg.pulse_width_bins  = nullw*cfg.n_ifft/cfg.n_tones;  % null-to-null, IFFT bins

% ------------------------------------------------- optional fields (defaults)
cfg.load_impedance     = gpr_cfg_value(cfg, 'load_impedance', 50);
% PA compression point and the drive level that lands pa_backoff_dB above it.
cfg.pa_backoff_dB      = gpr_cfg_value(cfg, 'pa_backoff_dB', 3);
cfg.pa_p1dB_dBm        = gpr_cfg_value(cfg, 'pa_p1dB_dBm', ...
    cfg.dac_power_dBm + cfg.pa_gain_dB + cfg.pa_backoff_dB);
cfg.predriver_gain_dB  = gpr_cfg_value(cfg, 'predriver_gain_dB', ...
    cfg.pa_p1dB_dBm + cfg.pa_backoff_dB - cfg.dac_power_dBm - cfg.pa_gain_dB);
cfg.tone_dwell_time    = gpr_cfg_value(cfg, 'tone_dwell_time', 50e-6);
cfg.tone_bandwidth     = 1/cfg.tone_dwell_time;
cfg.bg_remove_modes    = gpr_cfg_value(cfg, 'bg_remove_modes', 1);
cfg.cfar_estimator     = lower(gpr_cfg_value(cfg, 'cfar_estimator', 'log'));
if ~any(strcmp(cfg.cfar_estimator, {'log','mean'}))
    bad('cfar_estimator must be ''log'' (robust, default) or ''mean'' (classical CA-CFAR).');
    cfg.cfar_estimator = 'log';
end
cfg.cfar_guard         = gpr_cfg_value(cfg, 'cfar_guard', 4);
cfg.cfar_train         = gpr_cfg_value(cfg, 'cfar_train', 8);
cfg.migration_aperture = gpr_cfg_value(cfg, 'migration_aperture', 1.0);
cfg.tx_taper_percent   = gpr_cfg_value(cfg, 'tx_taper_percent', 0);
cfg.rx_cable_delay     = gpr_cfg_value(cfg, 'rx_cable_delay', 5e-9);
cfg.cal_ripple_depth   = gpr_cfg_value(cfg, 'cal_ripple_depth', 0.05);
cfg.cal_ripple_cycles  = gpr_cfg_value(cfg, 'cal_ripple_cycles', 3);
cfg.beam_half_angle_tan= gpr_cfg_value(cfg, 'beam_half_angle_tan', tan(pi/6));
cfg.footprint_min      = gpr_cfg_value(cfg, 'footprint_min', 0.30);
cfg.noise_seed         = gpr_cfg_value(cfg, 'noise_seed', 0);
cfg.sim_stop_time      = gpr_cfg_value(cfg, 'sim_stop_time', 0);   % one-shot batch

if cfg.cfar_guard < 1 || cfg.cfar_train < 1
    bad('cfar_guard and cfar_train must be >= 1 cell.');
end
% The guard band has to cover the compressed pulse, otherwise the target's
% own mainlobe leaks into the training ring and CFAR masks itself.
if cfg.cfar_guard < cfg.cfar_guard_min
    bad('cfar_guard (%d cells) must cover the -3 dB half-width of the windowed pulse (%d cells).', ...
        cfg.cfar_guard, cfg.cfar_guard_min);
end
if cfg.cfar_guard >= cfg.cfar_train
    bad('cfar_guard must be smaller than cfar_train (the guard window sits inside the training window).');
end
cfg.cfar_n_ref = (2*(cfg.cfar_guard+cfg.cfar_train)+1)^2 - (2*cfg.cfar_guard+1)^2;
% Detection threshold: power ratio a cell must exceed, and the same number in
% dB (used by check_physics_feasibility to compare against the link budget).
if strcmp(cfg.cfar_estimator, 'log')
    cfg.cfar_alpha = log(1/cfg.cfar_pfa);
else
    cfg.cfar_alpha = cfg.cfar_n_ref*(cfg.cfar_pfa^(-1/cfg.cfar_n_ref) - 1);
end
cfg.cfar_threshold_dB = 10*log10(cfg.cfar_alpha);
if cfg.cfar_n_ref < 8
    bad('cfar_train (%d) leaves only %d training cells; the noise estimate is meaningless.', ...
        cfg.cfar_train, cfg.cfar_n_ref);
end
if cfg.bg_remove_modes < 0 || cfg.bg_remove_modes > 5
    bad('bg_remove_modes must be in [0, 5] (got %g).', cfg.bg_remove_modes);
end

% Migration sampling: the first Fresnel zone at the shallowest target,
% sqrt(lambda_c*z/2), must span at least one trace spacing.  Below that the
% diffraction hyperbola is sampled by a single trace, B12 has no aperture to
% work with and the target keeps an unfocused tail that CFAR reports as a
% second detection next to every real one.
cfg.dx_trace = cfg.scan_length/max(cfg.n_positions - 1, 1);
lambda_c = cfg.soil_v/cfg.f_center;
cfg.fresnel_radius_min = sqrt(lambda_c*min(td)/2);
cfg.fresnel_traces = cfg.fresnel_radius_min/cfg.dx_trace;
if cfg.fresnel_traces < 1
    bad(['trace spacing (%.3f m) is too coarse to migrate at %.2f m depth: the ' ...
         'Fresnel zone is %.3f m (%.2f traces). Increase n_positions.'], ...
        cfg.dx_trace, min(td), cfg.fresnel_radius_min, cfg.fresnel_traces);
end

% Deepest target must fit in the unambiguous range and in the IFFT window.
unambig = v/(2*cfg.delta_f);
ifft_depth = cfg.n_ifft*cfg.dr_bin;
if max(td) > 0.9*unambig
    bad('deepest target (%.2f m) exceeds 90%% of the unambiguous depth (%.2f m); increase n_tones.', ...
        max(td), unambig);
end
if max(td) > 0.95*ifft_depth
    bad('deepest target (%.2f m) exceeds the IFFT window (%.2f m); increase n_ifft.', ...
        max(td), ifft_depth);
end
if cfg.depth_max > ifft_depth
    bad('depth_max (%.2f m) is outside the IFFT window (%.2f m).', cfg.depth_max, ifft_depth);
end
end
