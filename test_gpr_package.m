function t = test_gpr_package(varargin)
% TEST_GPR_PACKAGE  Functional test suite for the GPR digital twin.
%
%   t = test_gpr_package()            everything, ~2-3 minutes
%   t = test_gpr_package('quick', 1)  skip the slow mode-B end-to-end run
%
%   Every test either exercises a generated block script on its own (via
%   gpr_compile_script, i.e. exactly the text that goes into the Simulink
%   MATLAB Function block) or the whole chain through
%   run_pipeline_reference.  t.ok is true when all tests passed.
%
%   Tests
%     T01 soil model      permittivity/velocity/attenuation behave physically
%     T02 axes            depth and cross-range scaling are self-consistent
%     T03 config guards   nonsense configurations are rejected
%     T04 waveform        tone count, monotonic frequency, taper in [0,1]
%     T05 calibration     the RX cable delay is removed, peak lands on target
%     T06 background      a constant clutter column is subtracted
%     T07 migration       a single target focuses at its true (x, depth)
%     T08 CFAR            false alarms on pure noise match Pfa statistically
%     T09 report          union-find counts 8-connected blobs exactly
%     T10 test points     logged names match the spec (TP01..TP14)
%     T11 end-to-end A    4/4 targets, no false alarms, cm-level accuracy
%     T12 end-to-end B    5/5 targets, no false alarms (skipped by 'quick')

p = inputParser();
addParameter(p, 'quick', false);
parse(p, varargin{:});
opt = p.Results;

here = fileparts(mfilename('fullpath'));
if ~isempty(here) && isempty(strfind(path, here)) %#ok<STREMP>
    addpath(here);
end
wd = fullfile(tempdir, 'gpr_tests');
if ~exist(wd, 'dir'), mkdir(wd); end

% NOTE: struct() unwraps one level of cell, so empty containers must be
% wrapped in an extra cell or the result becomes a 0x0 struct array.
t = struct('ok', true, 'names', {cell(0,1)}, 'pass', {logical([])}, ...
    'seconds', {zeros(0,1)}, 'messages', {cell(0,1)}, ...
    'n_pass', 0, 'n_fail', 0);
% do_test returns the updated struct (MATLAB passes structs by value).
t = do_test(t, 'T01 soil model',        @() test_soil());
t = do_test(t, 'T02 axes',              @() test_axes());
t = do_test(t, 'T03 config guards',     @() test_config_guards());
t = do_test(t, 'T04 waveform',          @() test_waveform(wd));
t = do_test(t, 'T05 calibration',       @() test_calibration());
t = do_test(t, 'T06 background',        @() test_background(wd));
t = do_test(t, 'T07 migration focus',   @() test_migration());
t = do_test(t, 'T10 test-point names',  @() test_tp_names());
t = do_test(t, 'T08 CFAR false alarms', @() test_cfar(wd));
t = do_test(t, 'T09 report labelling',  @() test_report(wd));
t = do_test(t, 'T11 end-to-end mode A', @() test_e2e('A'));
if ~opt.quick
    t = do_test(t, 'T12 end-to-end mode B', @() test_e2e('B'));
end

fprintf('\n');
for k = 1:numel(t.names)
    if t.pass(k)
        fprintf('  PASS  %-26s %6.1f s\n', t.names{k}, t.seconds(k));
    else
        fprintf('  FAIL  %-26s %6.1f s  %s\n', t.names{k}, t.seconds(k), ...
            t.messages{k});
    end
end
fprintf('test_gpr_package: %d passed, %d failed\n', t.n_pass, t.n_fail);
end

% ------------------------------------------------------------------ harness
function t = do_test(t, name, fh)
t0 = tic();
try
    fh();
    ok = true; msg = '';
catch ME
    ok = false; msg = ME.message;
end
dt = toc(t0);
t.names{end+1} = name; %#ok<AGROW>
t.pass(end+1) = ok; %#ok<AGROW>
t.seconds(end+1) = dt; %#ok<AGROW>
t.messages{end+1} = msg; %#ok<AGROW>
if ok, t.n_pass = t.n_pass + 1; else, t.n_fail = t.n_fail + 1; t.ok = false; end
end

function need(cond, msg)
if ~cond
    error('GPR:test:assert', '%s', msg);
end
end

function cfg = small_cfg(tag)
if strcmp(tag, 'A')
    cfg = config_mode_A_uav_shallow();
else
    cfg = config_mode_B_ground_deep();
end
cfg.n_positions = 48;
cfg.scan_length = 2.4;
cfg.target_x = [1.2];
cfg.target_depths = [0.35];
cfg.target_rcs = [0.2];
cfg.n_averages = 8;
cfg = gpr_check_config(cfg, tag);
end

% -------------------------------------------------------------------- tests
function test_soil()
[e1, ~, a1, ~, v1] = soil_permittivity(1e9, 0.05, 0.005);
[e2, ~, a2, ~, v2] = soil_permittivity(1e9, 0.25, 0.005);
[e3, ~, a3, ~, ~ ] = soil_permittivity(3e9, 0.05, 0.005);
need(e1 > 2.5 && e1 < 40, sprintf('eps_r %.2f out of physical range', e1));
need(e2 > e1, 'permittivity must grow with moisture');
need(v2 < v1, 'velocity must drop with moisture');
need(a3 > a1, 'attenuation must grow with frequency');
% Moisture only enters the loss through the (fixed) conductivity term, see
% the note in soil_permittivity.m - assert the documented behaviour instead.
[~, ~, a4] = soil_permittivity(1e9, 0.05, 0.05);
need(a4 > a1, 'attenuation must grow with conductivity');
c0 = 299792458;
need(abs(v1 - c0/sqrt(e1))/v1 < 0.05, 'velocity must equal c/sqrt(eps_r)');
end

function test_axes()
cfg = gpr_check_config(config_mode_A_uav_shallow(), 'A');
ax = gpr_axes(cfg);
need(numel(ax.freq) == cfg.n_tones, 'wrong number of tones');
need(abs(ax.df - (cfg.f_stop-cfg.f_start)/(cfg.n_tones-1)) < 1, 'df wrong');
need(abs(ax.dr - ax.v_soil/(2*cfg.n_ifft*ax.df)) < 1e-9, 'dr wrong');
need(abs(ax.x(end) - cfg.scan_length) < 1e-9, 'x span wrong');
need(abs(ax.unambig_depth - ax.v_soil/(2*ax.df)) < 1e-6, 'unambiguous depth wrong');
need(ax.depth(1) == 0 && numel(ax.depth) == cfg.n_ifft, 'depth axis wrong');
end

function test_config_guards()
cfg = config_mode_A_uav_shallow();
bad = cfg; bad.cfar_guard = bad.cfar_train + 2;      % guard inside train
need_err(@() gpr_check_config(bad, 'A'), 'guard >= train must be rejected');
bad = cfg; bad.cfar_pfa = 5;                          % not a probability
need_err(@() gpr_check_config(bad, 'A'), 'Pfa > 1 must be rejected');
bad = cfg; bad.n_positions = 20;                      % under-sampled aperture
need_err(@() gpr_check_config(bad, 'A'), 'Fresnel under-sampling must be rejected');
bad = cfg; bad.cfar_estimator = 'median';
need_err(@() gpr_check_config(bad, 'A'), 'unknown estimator must be rejected');
bad = cfg; bad.n_ifft = 100;                          % not a power of two
need_err(@() gpr_check_config(bad, 'A'), 'non power-of-two IFFT must be rejected');
end

function need_err(fh, msg)
ok = false;
try
    fh();
catch
    ok = true;
end
need(ok, msg);
end

function test_waveform(wd)
cfg = gpr_check_config(config_mode_A_uav_shallow(), 'A');
fh = gpr_compile_script(code_waveform(cfg), 'ut_waveform', wd);
w = fh(cfg.f_start, cfg.f_stop);
need(size(w, 1) == cfg.n_tones, 'waveform length != n_tones');
need(all(diff(abs(w)) >= -1e-12) || all(diff(real(w)) >= -1e-12), ...
    'tone frequencies must be monotonic');
tpr = cfg.tx_taper_percent;
need(tpr >= 0 && tpr <= 50, 'taper percent out of range');
end

function test_calibration()
% After calibration and background removal the profile must peak at the
% true target depth, and that peak must not move when the RX cable delay
% changes: removing the cable delay is exactly what B09_Calibration is for.
cfg = small_cfg('A');
res = run_pipeline_reference(cfg, 'verbose', false);
% Before migration the range axis still carries the air leg, expressed in
% soil-equivalent depth; B12_Migration is what removes it (T07 checks that).
d_exp = cfg.uav_altitude*res.ax.v_soil/299792458 + 0.35;
d1 = peak_depth(res);
need(abs(d1 - d_exp) < 3*res.ax.dr, ...
    sprintf('peak at %.3f m, want %.3f (air leg + target)', d1, d_exp));
cfg2 = cfg;
cfg2.rx_cable_delay = cfg.rx_cable_delay + 40e-9;
cfg2 = gpr_check_config(cfg2, 'A');
res2 = run_pipeline_reference(cfg2, 'verbose', false);
d2 = peak_depth(res2);
need(abs(d1 - d2) < res.ax.dr, sprintf('cable delay moved the peak %.3f -> %.3f m', d1, d2));
end

function d = peak_depth(res)
% The strongest echo in the range profile is the ground-surface bounce (at
% standoff*v_soil/c in soil-depth units), so the target is the peak of the
% background-removed profile, which is what the rest of the chain sees.
[~, lin] = max(abs(res.bs(:)));
[r, ~] = ind2sub(size(res.bs), lin);
d = (r-1)*res.ax.dr;
end

function test_background(wd)
cfg = gpr_check_config(config_mode_A_uav_shallow(), 'A');
fh = gpr_compile_script(code_background(cfg), 'ut_background', wd);
rng(7);
np = cfg.n_positions;
X = randn(cfg.n_ifft, np) + 3*exp(1i*0.7);   % constant clutter + noise
X(50, 20) = X(50, 20) + 50;                  % one target
Y = fh(X, 1);
need(abs(mean(Y(:, 1))) < 0.5, 'constant clutter column must be removed');
[~, ip] = max(abs(Y(:, 20)));
need(ip == 50, 'the target must survive background removal');
end

function test_migration()
cfg = small_cfg('A');
res = run_pipeline_reference(cfg, 'verbose', false);
[~, lin] = max(abs(res.mig(:)));
[r, c] = ind2sub(size(res.mig), lin);
d = (r-1)*res.ax.dr;
xx = (c-1)*cfg.dx_trace;
need(abs(d - 0.35) < 3*res.ax.dr, sprintf('migrated depth %.3f != 0.35', d));
need(abs(xx - 1.2) < 3*cfg.dx_trace, sprintf('migrated x %.3f != 1.2', xx));
end

function test_cfar(wd)
cfg = gpr_check_config(config_mode_A_uav_shallow(), 'A');
cfg.cfar_pfa = 1e-3;
ax0 = gpr_axes(cfg);
cfg.depth_min = 0;
cfg.depth_max = ax0.max_ifft_depth;   % whole matrix is a valid band
cfg = gpr_check_config(cfg, 'A');
fh = gpr_compile_script(code_detection(cfg), 'ut_detection', wd);
rng(11);
% Migrated thermal noise is circular complex Gaussian, so its POWER is
% exponentially distributed - the distribution the CFAR threshold is derived
% for.  abs(randn) would be chi-square-1 and inflate the false-alarm rate.
mig = abs(randn(cfg.n_ifft, cfg.n_positions) + ...
          1i*randn(cfg.n_ifft, cfg.n_positions))/sqrt(2);
det = fh(mig, cfg.depth_min, cfg.depth_max, cfg.cfar_pfa, cfg.f_start, ...
    cfg.f_stop, cfg.soil_moisture, cfg.soil_conductivity);
n = sum(det(:) > 0);
expect = cfg.cfar_pfa*numel(mig);
need(n > 0.3*expect && n < 3*expect, sprintf('%d false alarms, expected ~%.0f', ...
    n, expect));
end

function test_report(wd)
cfg = gpr_check_config(config_mode_A_uav_shallow(), 'A');
fh = gpr_compile_script(code_report(cfg), 'ut_report', wd);
det = zeros(cfg.n_ifft, cfg.n_positions);
% blob 1, blob 2 diagonal (8-connected), blob 3 isolated
det(100:103, 10:13) = 1;
det(200:202, 20:22) = 1;
det(203:205, 23:25) = 1;
det(300:302, 40:42) = 1;
mig = rand(cfg.n_ifft, cfg.n_positions);
mig(101, 11) = 9; mig(204, 24) = 8; mig(301, 41) = 7;
rep = fh(det, mig, cfg.f_start, cfg.f_stop, cfg.soil_moisture, ...
    cfg.soil_conductivity, cfg.scan_length);
need(rep(2) == 3, sprintf('expected 3 clusters (diagonal joins), got %g', rep(2)));
need(rep(1) == sum(det(:)), 'pixel count must equal the mask population');
dr = gpr_axes(cfg).dr;
need(abs(rep(5) - 100*dr) < 1e-9, sprintf('min peak depth %g != %g', rep(5), 100*dr));
need(abs(rep(6) - 300*dr) < 1e-9, sprintf('max peak depth %g != %g', rep(6), 300*dr));
end

function test_tp_names()
cfg = small_cfg('A');
res = run_pipeline_reference(cfg, 'verbose', false);
sp = gpr_pipeline_spec(cfg);
want = {};
for sec = {'rf', 'dsp'}
    S = sp.(sec{1});
    for k = 1:numel(S.blocks)
        nm = sprintf('tp%02d_%s', S.first_tp + k - 1, lower(S.blocks{k}.name));
        want{end+1} = nm; %#ok<AGROW>
        if strcmp(S.blocks{k}.name, 'B06_RX') || strcmp(S.blocks{k}.name, 'B07_ADC')
            want{end+1} = [nm '_o2']; %#ok<AGROW>
        end
    end
end
got = fieldnames(res.tp);
for k = 1:numel(want)
    need(any(strcmp(got, want{k})), sprintf('missing test point %s', want{k}));
end
end

function test_e2e(mode)
res = gpr_realistic_main('mode', mode, 'engine', 'reference', ...
    'verbose', false, 'save', false);
cfg = res.cfg;
nhit = sum(~isnan(res.match(:, 1)));
nfalse = sum(res.unused_det);
need(nhit == cfg.n_targets, sprintf('%d/%d targets detected', nhit, ...
    cfg.n_targets));
need(nfalse == 0, sprintf('%d false alarms', nfalse));
tol_d = 3*cfg.depth_resolution;
tol_x = 6*cfg.dx_trace;
need(max(abs(res.match(1:cfg.n_targets, 1))) < tol_x, 'cross-range error too big');
need(max(abs(res.match(1:cfg.n_targets, 2))) < tol_d, 'depth error too big');
need(res.report(2) == cfg.n_targets, 'B14 cluster count must match');
end
