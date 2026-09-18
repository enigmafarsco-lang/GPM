function f = check_physics_feasibility(cfg, varargin)
% CHECK_PHYSICS_FEASIBILITY  Independent physics audit of a GPR configuration.
%
%   f = CHECK_PHYSICS_FEASIBILITY(cfg) re-derives the survey from first
%   principles - Dobson permittivity, the radar equation, thermal noise,
%   quantisation noise, sampling theory - and reports whether the configured
%   system can actually see the configured targets.  It does NOT trust the
%   numbers in cfg beyond the hardware description, so it catches the two
%   classic mistakes: a band/depth combination that is attenuated into the
%   noise, and a sampling grid that cannot represent the answer.
%
%   f = CHECK_PHYSICS_FEASIBILITY(cfg, 'verbose', false) suppresses printing.
%
%   Fields of the returned struct
%       .ok            true when no check failed
%       .errors        cell array of failure messages
%       .warnings      cell array of advisory messages
%       .lines         the printed report, one cell per line
%       .soil          eps_r, v, alpha_Np_per_m, alpha_dB_per_m, loss_tangent
%       .resolution    range_resolution_m, depth_resolution_m (windowed),
%                      x_resolution_m, fresnel_radius_m, dx_trace_m
%       .depth         unambiguous_m, ifft_window_m, deepest_target_m
%       .link          per-target rows: R_air, R_s, Pr_dBm, Pn_dBm,
%                      SNR_dB_single, SNR_dB_after_averaging, margin_dB
%       .dynamic_range surface_dBm, deepest_target_dBm, spread_dB,
%                      adc_range_dB, headroom_dB
%       .timing        tone_dwell_s, sweep_s, total_s
%       .cfar          n_resolution_cells, expected_false_alarms
%
%   The link budget uses exactly the propagation model of B04_Channel
%   (slant range in soil, Fresnel interface transmission, two-way
%   attenuation, lambda/((4 pi)^1.5 R^2) spreading) so the reported SNR is
%   the SNR the simulated chain will actually produce.  The previous version
%   of this file used a "-6 dB" fudge factor and ignored the air leg, which
%   made it agree with nothing.

p = inputParser();
addParameter(p, 'verbose', true, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
verbose = logical(p.Results.verbose);

cfg = gpr_check_config(cfg, cfg.mode);

c0   = 299792458;
kB   = 1.380649e-23;
T0   = 290;

f = struct();
f.errors = {};
f.warnings = {};

% ------------------------------------------------------------------- soil
[eps_r, eps_i, alpha, ~, v] = soil_permittivity(cfg.f_center, ...
    cfg.soil_moisture, cfg.soil_conductivity);
f.soil = struct('eps_r', eps_r, 'eps_i', eps_i, 'v', v, ...
    'alpha_Np_per_m', alpha, 'alpha_dB_per_m', alpha*20/log(10), ...
    'loss_tangent', eps_i/eps_r, 'f_center', cfg.f_center);

% ------------------------------------------------------------- resolution
lambda_c = v/cfg.f_center;
f.resolution = struct( ...
    'range_resolution_m',  cfg.range_resolution, ...
    'depth_resolution_m',  cfg.depth_resolution, ...
    'x_resolution_m',      max(cfg.dx_trace, lambda_c/2), ...
    'fresnel_radius_m',    cfg.fresnel_radius_min, ...
    'dx_trace_m',          cfg.dx_trace, ...
    'window',              cfg.range_window, ...
    'window_broadening',   cfg.window_broadening);

% ------------------------------------------------------------------ depth
unambig  = v/(2*cfg.delta_f);
ifft_win = cfg.n_ifft*cfg.dr_bin;
f.depth = struct('unambiguous_m', unambig, 'ifft_window_m', ifft_win, ...
    'deepest_target_m', max(cfg.target_depths), 'depth_min_m', cfg.depth_min, ...
    'depth_max_m', cfg.depth_max);

if max(cfg.target_depths) > 0.9*unambig
    f.errors{end+1} = sprintf(...
        'deepest target %.2f m exceeds 90%% of the unambiguous depth %.2f m', ...
        max(cfg.target_depths), unambig);
end
if max(cfg.target_depths) > 0.95*ifft_win
    f.errors{end+1} = sprintf(...
        'deepest target %.2f m exceeds 95%% of the IFFT window %.2f m', ...
        max(cfg.target_depths), ifft_win);
end
if numel(cfg.target_depths) > 1
    gap = min(diff(sort(cfg.target_depths)));
    if gap < cfg.depth_resolution
        f.warnings{end+1} = sprintf(...
            ['two targets are only %.2f m apart in depth, below the windowed ' ...
             'depth resolution %.2f m - they will merge into one blob'], ...
            gap, cfg.depth_resolution);
    end
end

% --------------------------------------------------------- noise floor
% Receiver noise power in the tone bandwidth, referred to the input of the
% ADC.  n_avg averages reduce it by n_avg (verified end to end by
% test_gpr_package.m: the injected noise rms is adc_noise/sqrt(n_avg)).
bw_tone  = cfg.tone_bandwidth;
nf_lin   = 10^(cfg.system_nf_dB/10);
Pn_W     = kB*T0*bw_tone*nf_lin;
Pn_dBm   = 10*log10(Pn_W) + 30;
navg     = max(cfg.n_averages, 1);

% Quantisation noise of the ADC over the same bandwidth.  A full-scale sine
% gives SNR = 6.02*ENOB + 1.76 dB over the Nyquist band; the processing gain
% of the sweep is accounted for separately by the compression gain below.
adc_snr_dB   = 6.02*cfg.adc_enob + 1.76;
f.noise = struct('tone_bandwidth_Hz', bw_tone, 'nf_dB', cfg.system_nf_dB, ...
    'thermal_dBm', Pn_dBm, 'adc_snr_dB', adc_snr_dB, 'n_averages', navg);

% ------------------------------------------------------------ link budget
n2    = sqrt(eps_r);
Gam   = (n2 - 1)/(n2 + 1);
T2    = (1 - Gam^2)^2;
eirp_W = 10^(cfg.eirp_dBm/10)/1000;
R_air  = max(cfg.uav_altitude, 0.01);
lam    = c0/cfg.f_center;

f.link = struct('x', {}, 'depth', {}, 'rcs', {}, 'R_air', {}, 'R_s', {}, ...
    'Pr_dBm', {}, 'Pn_dBm', {}, 'snr_single_dB', {}, 'snr_averaged_dB', {}, ...
    'margin_dB', {});
compression_gain_dB = 10*log10(cfg.n_tones);   % pulse-compression gain

for t = 1:cfg.n_targets
    % worst case for this target: the trace directly above it (shortest path)
    R_s   = cfg.target_depths(t);
    R_eff = R_air + R_s;
    % exactly the B04_Channel amplitude model, squared into power
    Pr_W  = eirp_W * cfg.target_rcs(t) * T2^2 * lam^2 ...
            * exp(-4*alpha*R_s) / ((4*pi)^3 * R_eff^4);
    Pr_dBm = 10*log10(max(Pr_W, 1e-30)) + 30;
    snr1   = Pr_dBm - Pn_dBm + compression_gain_dB;
    snrN   = snr1 + 10*log10(navg);
    f.link(t) = struct('x', cfg.target_x(t), 'depth', cfg.target_depths(t), ...
        'rcs', cfg.target_rcs(t), 'R_air', R_air, 'R_s', R_s, ...
        'Pr_dBm', Pr_dBm, 'Pn_dBm', Pn_dBm, 'snr_single_dB', snr1, ...
        'snr_averaged_dB', snrN, 'margin_dB', snrN - cfg.cfar_threshold_dB);
    if snrN < cfg.cfar_threshold_dB
        f.errors{end+1} = sprintf(...
            ['target %d (%.1f m, %.1f m, %.3g m^2) has %.1f dB SNR after ' ...
             '%d averages, below the %.1f dB CFAR threshold'], ...
            t, cfg.target_x(t), R_s, cfg.target_rcs(t), snrN, navg, ...
            cfg.cfar_threshold_dB);
    elseif snrN < cfg.cfar_threshold_dB + 6
        f.warnings{end+1} = sprintf(...
            'target %d has only %.1f dB of SNR margin over the CFAR threshold', ...
            t, snrN - cfg.cfar_threshold_dB);
    end
end

% --------------------------------------------------------- dynamic range
% The surface bounce is the one echo that is NOT a point target: a plane
% interface fills the whole beam, so B04_Channel models it as the bare
% Fresnel coefficient with no 1/R^2 spreading (a point-target term would
% diverge for a 2 cm ground clearance).  The dynamic-range check has to use
% the same convention, otherwise it reports a spread the ADC never sees.
Psurf_W = eirp_W*Gam^2;
surface_dBm = 10*log10(max(Psurf_W, 1e-30)) + 30;
deepest_dBm = f.link(end).Pr_dBm;
spread_dB   = surface_dBm - deepest_dBm;
adc_range   = adc_snr_dB;
f.dynamic_range = struct('surface_dBm', surface_dBm, ...
    'deepest_target_dBm', deepest_dBm, 'spread_dB', spread_dB, ...
    'adc_range_dB', adc_range, 'headroom_dB', adc_range - spread_dB);
if spread_dB > adc_range
    f.errors{end+1} = sprintf(...
        ['the surface bounce is %.0f dB above the deepest target but the ADC ' ...
         'only offers %.0f dB: the deep target is buried in quantisation ' ...
         'noise. Increase adc_enob or reduce the standoff.'], spread_dB, adc_range);
elseif spread_dB > adc_range - 6
    f.warnings{end+1} = sprintf(...
        'only %.1f dB of ADC headroom left after the %.0f dB surface/target spread', ...
        adc_range - spread_dB, spread_dB);
end

% ---------------------------------------------------------------- timing
sweep_s = cfg.n_tones*cfg.tone_dwell_time;
total_s = sweep_s*cfg.n_positions*navg;
f.timing = struct('tone_dwell_s', cfg.tone_dwell_time, 'sweep_s', sweep_s, ...
    'per_position_s', sweep_s*navg, 'total_s', total_s);
if total_s > 3600
    f.warnings{end+1} = sprintf('the survey takes %.1f min', total_s/60);
end

% ------------------------------------------------------------------ CFAR
n_cells = max(1, round((cfg.depth_max - cfg.depth_min)/cfg.depth_resolution)) ...
        * cfg.n_positions;
f.cfar = struct('pfa', cfg.cfar_pfa, 'estimator', cfg.cfar_estimator, ...
    'n_resolution_cells', n_cells, ...
    'expected_false_alarms', cfg.cfar_pfa*n_cells, ...
    'threshold_dB', cfg.cfar_threshold_dB, 'guard', cfg.cfar_guard, ...
    'train', cfg.cfar_train);
if cfg.cfar_pfa*n_cells > 1
    f.warnings{end+1} = sprintf(...
        ['cfar_pfa = %.0e over %d resolution cells gives %.1f expected false ' ...
         'alarms per section; lower cfar_pfa'], ...
        cfg.cfar_pfa, n_cells, cfg.cfar_pfa*n_cells);
end
if cfg.cfar_guard < cfg.cfar_guard_min
    f.errors{end+1} = sprintf(...
        'cfar_guard (%d) does not cover the compressed pulse (%d cells)', ...
        cfg.cfar_guard, cfg.cfar_guard_min);
end

% ---------------------------------------------------------------- report
f.ok = isempty(f.errors);
f.lines = render(f, cfg);
if verbose
    fprintf('%s\n', f.lines{:});
end
end

% ------------------------------------------------------------------ report
function lines = render(f, cfg)
lines = {};
lines{end+1} = sprintf('=== physics feasibility: %s (%s) ===', cfg.mode, cfg.description); %#ok<AGROW>
lines{end+1} = sprintf('band %.3g-%.3g MHz, %d tones, %d positions over %.1f m, standoff %.2f m', ...
    cfg.f_start/1e6, cfg.f_stop/1e6, cfg.n_tones, cfg.n_positions, cfg.scan_length, cfg.uav_altitude); %#ok<AGROW>
lines{end+1} = sprintf('soil: moisture %.3f m^3/m^3, sigma %.3g S/m -> eps_r %.2f, v %.3g m/s, %.2f dB/m', ...
    cfg.soil_moisture, cfg.soil_conductivity, f.soil.eps_r, f.soil.v, f.soil.alpha_dB_per_m); %#ok<AGROW>
lines{end+1} = sprintf('resolution: range %.4g m, depth (windowed, %s) %.4g m, trace spacing %.4g m', ...
    f.resolution.range_resolution_m, f.resolution.window, ...
    f.resolution.depth_resolution_m, f.resolution.dx_trace_m); %#ok<AGROW>
lines{end+1} = sprintf('depth: unambiguous %.3g m, IFFT window %.3g m, deepest target %.3g m', ...
    f.depth.unambiguous_m, f.depth.ifft_window_m, f.depth.deepest_target_m); %#ok<AGROW>
lines{end+1} = sprintf('noise: thermal %.1f dBm in %.3g Hz, NF %.1f dB, ADC %.1f dB, %d averages', ...
    f.noise.thermal_dBm, f.noise.tone_bandwidth_Hz, f.noise.nf_dB, ...
    f.noise.adc_snr_dB, f.noise.n_averages); %#ok<AGROW>
lines{end+1} = 'link budget (worst case, antenna directly above the target):';
lines{end+1} = sprintf('  %-4s %7s %7s %8s %10s %10s %9s', ...
    'tgt', 'x [m]', 'd [m]', 'rcs m^2', 'Pr [dBm]', 'SNR [dB]', 'margin'); %#ok<AGROW>
for t = 1:numel(f.link)
    L = f.link(t);
    lines{end+1} = sprintf('  %-4d %7.2f %7.2f %8.3g %10.1f %10.1f %+8.1f dB', ...
        t, L.x, L.depth, L.rcs, L.Pr_dBm, L.snr_averaged_dB, L.margin_dB); %#ok<AGROW>
end
lines{end+1} = sprintf('dynamic range: surface %.1f dBm, deepest target %.1f dBm -> %.1f dB spread, ADC offers %.1f dB', ...
    f.dynamic_range.surface_dBm, f.dynamic_range.deepest_target_dBm, ...
    f.dynamic_range.spread_dB, f.dynamic_range.adc_range_dB); %#ok<AGROW>
lines{end+1} = sprintf('timing: %.1f ms per sweep, %.2f s per position, %.1f s total', ...
    f.timing.sweep_s*1e3, f.timing.per_position_s, f.timing.total_s); %#ok<AGROW>
lines{end+1} = sprintf('CFAR: %s estimator, Pfa %.0e over %d resolution cells -> %.2g expected false alarms', ...
    f.cfar.estimator, f.cfar.pfa, f.cfar.n_resolution_cells, ...
    f.cfar.expected_false_alarms); %#ok<AGROW>
for k = 1:numel(f.warnings)
    lines{end+1} = sprintf('WARNING: %s', f.warnings{k}); %#ok<AGROW>
end
for k = 1:numel(f.errors)
    lines{end+1} = sprintf('ERROR:   %s', f.errors{k}); %#ok<AGROW>
end
if f.ok
    lines{end+1} = 'result: FEASIBLE';
else
    lines{end+1} = 'result: NOT FEASIBLE as configured';
end
end
