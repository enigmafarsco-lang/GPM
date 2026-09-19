function v = validate_package(varargin)
% VALIDATE_PACKAGE  Static checks of the GPR package, no Simulink needed.
%
%   v = validate_package()
%   v = validate_package('build', true)   also build + update the .slx when
%                                         Simulink is installed
%
%   Checks, per configuration (mode A and mode B):
%     - every source file of the package is present
%     - every code_*.m declares a function with the same name as its file
%       (MATLAB ignores a file whose first function name differs, which is
%        how the original code_generators.m silently produced nothing)
%     - the pipeline spec resolves: every block input points at an existing
%        block/constant of the same section, on an existing output port
%     - every generated block script parses and its signature matches the
%        number of wires the spec attaches to it
%     - gpr_check_config accepts the configuration and derives all fields
%     - check_physics_feasibility reports the configuration feasible
%   Plus, once:
%     - the two shipped configurations disagree where they should (band,
%        standoff, estimator) so they really are two modes
%     - the reference engine can execute the chain end to end (fast smoke
%        run with a tiny scene, not the full detection test)
%
%   v.ok is true when no check failed; v.lines is the printable report.

p = inputParser();
addParameter(p, 'build', false);
addParameter(p, 'codegen', false);
parse(p, varargin{:});
opt = p.Results;

here = fileparts(mfilename('fullpath'));
if ~isempty(here) && isempty(strfind(path, here)) %#ok<STREMP>
    addpath(here);
end
fprintf('GPM package version %s (%s)\n', gpr_version(), here);

v = struct('ok', true, 'n_checks', 0, 'n_fail', 0, 'lines', {{}}, ...
    'failures', {{}});

% --------------------------------------------------------------- file list
files = {'gpr_pipeline_spec.m', 'gpr_check_config.m', 'gpr_axes.m', ...
    'gpr_cfg_value.m', 'gpr_parse_script.m', 'gpr_compile_script.m', ...
    'gpr_extract_results.m', 'gpr_pulse_width.m', 'gpr_range_window.m', ...
    'soil_permittivity.m', 'run_pipeline_reference.m', ...
    'check_physics_feasibility.m', 'build_realistic_model.m', ...
    'gpr_tp_from_simout.m', 'gpr_realistic_main.m', 'gpr_realistic_gui.m', ...
    'config_mode_A_uav_shallow.m', 'config_mode_B_ground_deep.m', ...
    'code_environment.m', 'code_waveform.m', 'code_tx.m', 'code_channel.m', ...
    'code_coupling.m', 'code_rx.m', 'code_adc.m', 'code_averaging.m', ...
    'code_calibration.m', 'code_rangeproc.m', 'code_background.m', ...
    'code_migration.m', 'code_detection.m', 'code_report.m'};
for k = 1:numel(files)
    v = check(v, exist(fullfile(here, files{k}), 'file') == 2 || ...
        exist(files{k}, 'file') == 2, sprintf('file present: %s', files{k}));
end
% The monolithic generator file must be gone: MATLAB would resolve
% code_environment() etc. from it only if the names matched, and they never
% did, so its presence is at best dead weight and at worst shadowing.
v = check(v, ~(exist(fullfile(here, 'code_generators.m'), 'file') == 2), ...
    'obsolete code_generators.m removed');

% ------------------------------------------- function name == file name
gens = {'code_environment', 'code_waveform', 'code_tx', 'code_channel', ...
    'code_coupling', 'code_rx', 'code_adc', 'code_averaging', ...
    'code_calibration', 'code_rangeproc', 'code_background', ...
    'code_migration', 'code_detection', 'code_report'};
for k = 1:numel(gens)
    f = which(gens{k});
    ok = ~isempty(f);
    if ok
        txt = fileread(f);
        ln = strsplit(txt, newline);
        ln = strtrim(ln{1});
        % First line of a MATLAB file must be its function signature; match
        % both "function out = name(...)" and "function name(...)".
        m = regexp(ln, '=\s*(\w+)\s*\(', 'tokens', 'once');
        if isempty(m)
            m = regexp(ln, '^function\s+(\w+)', 'tokens', 'once');
        end
        ok = ~isempty(m) && strcmp(m{1}, gens{k});
    end
    v = check(v, ok, sprintf('%s: first function name matches file name', gens{k}));
end

% ---------------------------------------------- one installation on the path
off = gpr_path_sanity();
v = check(v, isempty(off), sprintf('single package installation on path (%s)', ...
    strjoin(off, '; ')));

% ------------------------------------------------------------ spec + config
modes = {'A', 'B'};
cfgs = struct();
for m = 1:numel(modes)
    tag = modes{m};
    try
        if strcmp(tag, 'A')
            cfg = gpr_check_config(config_mode_A_uav_shallow(), 'A');
        else
            cfg = gpr_check_config(config_mode_B_ground_deep(), 'B');
        end
        cfgs.(tag) = cfg;
        v = check(v, true, sprintf('mode %s: gpr_check_config accepts', tag));
    catch ME
        v = check(v, false, sprintf('mode %s: gpr_check_config accepts (%s)', ...
            tag, ME.message));
        continue;
    end
    for fld = {'n_tones', 'n_ifft', 'n_positions', 'dx_trace', 'cfar_n_ref', ...
            'cfar_alpha', 'cfar_threshold_dB', 'depth_resolution', ...
            'fresnel_traces', 'pulse_width_bins'}
        v = check(v, isfield(cfg, fld{1}), ...
            sprintf('mode %s: derived field %s', tag, fld{1}));
    end
    f = check_physics_feasibility(cfg, 'verbose', false);
    v = check(v, f.ok, sprintf('mode %s: physics feasible (%s)', tag, ...
        strjoin(f.errors, '; ')));

    % spec wiring
    ok = true; msg = '';
    try
        sp = gpr_pipeline_spec(cfg);
        for sec = {'rf', 'dsp'}
            S = sp.(sec{1});
            names = {};
            nout = containers.Map();
            for j = 1:numel(S.blocks)
                names{end+1} = S.blocks{j}.name; %#ok<AGROW>
            end
            for j = 1:numel(S.consts)
                names{end+1} = S.consts{j}.name; %#ok<AGROW>
            end
            if strcmp(sec{1}, 'dsp')
                for j = 1:numel(S.inports)
                    names{end+1} = S.inports{j}.name; %#ok<AGROW>
                end
            end
            for j = 1:numel(S.blocks)
                blk = S.blocks{j};
                scr = feval(blk.gen, cfg);
                info = gpr_parse_script(scr);
                if info.nin ~= numel(blk.inputs)
                    ok = false;
                    msg = sprintf('%s: script takes %d inputs, spec wires %d', ...
                        blk.name, info.nin, numel(blk.inputs));
                end
                nout(blk.name) = info.nout;
                for q = 1:numel(blk.inputs)
                    ref = blk.inputs{q};
                    [bn, prt] = strtok(ref, ':');
                    prt = str2double(strrep(prt, ':', ''));
                    if isempty(prt) || isnan(prt), prt = 1; end
                    if ~any(strcmp(names, bn))
                        ok = false;
                        msg = sprintf('%s: input %d refers to unknown block %s', ...
                            blk.name, q, bn);
                    elseif isKey(nout, bn) && prt > nout(bn)
                        ok = false;
                        msg = sprintf('%s: input %d uses port %d of %s (has %d)', ...
                            blk.name, q, prt, bn, nout(bn));
                    end
                end
            end
        end
    catch ME
        ok = false; msg = ME.message;
    end
    v = check(v, ok, sprintf('mode %s: spec wiring resolves (%s)', tag, msg));

    % the analytic I/O size table must agree with the spec wiring
    ok = true; msg = '';
    try
        io = gpr_block_io_sizes(cfg);
        if strcmp(tag, 'A'), ioA = io; end
        sp2 = gpr_pipeline_spec(cfg);
        for sec = {'rf', 'dsp'}
            S = sp2.(sec{1});
            for j = 1:numel(S.blocks)
                b = S.blocks{j};
                e = io.(b.name);
                if numel(e.in_sz) ~= numel(b.inputs)
                    ok = false;
                    msg = sprintf('%s: size table has %d inputs, spec wires %d', ...
                        b.name, numel(e.in_sz), numel(b.inputs));
                elseif numel(e.out_sz) ~= numel(e.out_cx)
                    ok = false;
                    msg = sprintf('%s: size/complexity list mismatch', b.name);
                elseif any(cellfun(@(z) any(z <= 0), [e.in_sz e.out_sz]))
                    ok = false;
                    msg = sprintf('%s: non-positive port size', b.name);
                end
            end
        end
    catch ME
        ok = false; msg = ME.message;
    end
    v = check(v, ok, sprintf('mode %s: analytic I/O size table matches spec (%s)', tag, msg));
end

% ------------------------- release fingerprints (mixed-vintage detection)
% An extract that overlays a new zip on an old folder leaves some files at
% the old version while gpr_version claims the new one; these markers are
% the fixes themselves, so a missing marker means a stale file.
fp = { 'code_background', 'A = complex(rp)',      1; ...
       'code_migration',  'acc = complex(0)',     1; ...
       'code_adc',        'snr_db = 6.02',         1; ...
       'code_adc',        'noise_std = reshape',  0; ...
       'code_rangeproc',  'w = [',                 1 };
ok = true; msg = '';
for k = 1:size(fp, 1)
    scr = feval(fp{k,1}, cfgs.A);
    has = ~isempty(strfind(scr, fp{k,2})); %#ok<STREMP>
    if has ~= logical(fp{k,3})
        ok = false;
        msg = sprintf('%s %s marker ''%s''', msg, fp{k,1}, fp{k,2});
    end
end
v = check(v, ok, sprintf('release fingerprints present (%s)', msg));

% --------------------- probed I/O must agree with the analytic table
ok = true; msg = '';
for kk = 1:numel(gens)
    bn = blk_of_gen(gens{kk});
    if isempty(bn), continue; end
    e = ioA.(bn);
    for q = 1:numel(e.out_sz)
        if ~isequal(e.out_sz{q}, e.table_sz{q}) || ~strcmp(e.out_cx{q}, e.table_cx{q})
            ok = false;
            msg = sprintf('%s %s: probe [%dx%d %s] vs table [%dx%d %s];', msg, bn, ...
                e.out_sz{q}(1), e.out_sz{q}(2), e.out_cx{q}, ...
                e.table_sz{q}(1), e.table_sz{q}(2), e.table_cx{q});
        end
    end
end
v = check(v, ok, sprintf('probed I/O equals analytic table (%s)', msg));

% ------------------------------------------- generated scripts: coder-safe
% A persistent variable inside a MATLAB Function block is a parse error for
% MATLAB Coder unless every read is an isempty guard; the package bakes such
% constants in instead, so no generated script may contain one.
ok = true; msg = '';
for k = 1:numel(gens)
    scr = feval(gens{k}, cfgs.A);
    if ~isempty(regexp(scr, '(?m)^\s*persistent\b', 'once'))
        ok = false;
        msg = sprintf('%s uses persistent', gens{k});
    end
end
v = check(v, ok, sprintf('no generated block script uses persistent (%s)', msg));

% the two modes must really differ
if isfield(cfgs, 'A') && isfield(cfgs, 'B')
    a = cfgs.A; b = cfgs.B;
    v = check(v, a.f_start > b.f_stop, 'modes use non-overlapping bands');
    v = check(v, a.uav_altitude > 10*b.uav_altitude, 'modes use different standoffs');
    v = check(v, ~strcmp(a.cfar_estimator, b.cfar_estimator), ...
        'modes use different CFAR estimators (see config comments)');
end

% ------------------------------------------------- optional Simulink build
if opt.build
    if ~isempty(which('sim')) && license('test', 'Simulink') %#ok<LICTST>
        try
            mdl = build_realistic_model(cfgs.B, 'verbose', false);
            v = check(v, ~isempty(mdl), 'Simulink model builds and updates');
        catch ME
            v = check(v, false, sprintf('Simulink model builds (%s)', ME.message));
        end
    else
        v = note(v, 'Simulink not installed - skipped the model build check.');
    end
else
    v = note(v, 'Model build not requested (''build'', true to include it).');
end

% ------------------------------------- optional: MATLAB Coder parse check
% This is the same parser Simulink runs on every MATLAB Function block, but
% offline and per block, so a Coder-level type error is caught without ever
% opening Simulink.  Needs MATLAB Coder; skipped silently without it.
if opt.codegen
    if isempty(which('codegen'))
        v = note(v, 'codegen requested but MATLAB Coder is not installed - skipped.');
    else
        cfg = cfgs.A;
        io = gpr_block_io_sizes(cfg);
        sp = gpr_pipeline_spec(cfg);
        wd = fullfile(tempdir, 'gpr_codegen_check');
        if ~exist(wd, 'dir'), mkdir(wd); end
        for sec = {'rf', 'dsp'}
            S = sp.(sec{1});
            for j = 1:numel(S.blocks)
                b = S.blocks{j};
                nm = ['cgchk_' lower(b.name)];
                ok = true; msg = '';
                try
                    gpr_compile_script(feval(b.gen, cfg), nm, wd);
                    e = io.(b.name);
                    args = cell(1, numel(e.in_sz));
                    for q = 1:numel(e.in_sz)
                        base = 1;
                        if strcmp(e.in_cx{q}, 'Complex')
                            base = complex(1);
                        end
                        args{q} = coder.typeof(base, e.in_sz{q}, false(e.in_sz{q}));
                    end
                    codegen(nm, '-args', args, '-o', fullfile(wd, 'out'));
                catch ME
                    ok = false; msg = ME.message;
                end
                v = check(v, ok, sprintf('mode A: %s parses under MATLAB Coder (%s)', ...
                    b.name, msg));
            end
        end
    end
else
    v = note(v, 'MATLAB Coder parse check not requested (''codegen'', true).');
end

% ------------------------------------------------------------- smoke run
try
    cfg = cfgs.A;
    cfg.n_positions = 24;
    cfg.scan_length = 1.2;
    cfg.target_x = [0.4 0.8];
    cfg.target_depths = [0.2 0.4];
    cfg.target_rcs = [0.1 0.1];
    cfg.n_averages = 4;
    cfg = gpr_check_config(cfg, 'A');
    r = run_pipeline_reference(cfg, 'verbose', false);
    v = check(v, numel(r.report) == 6 && r.report(1) >= 0, ...
        'reference engine smoke run produces a B14 report');
catch ME
    v = check(v, false, sprintf('reference engine smoke run (%s)', ME.message));
end

fprintf('%s\n', strjoin(v.lines, sprintf('\n')));
if v.ok
    fprintf('validate_package: ALL %d CHECKS PASSED (package version %s)\n', ...
        v.n_checks, gpr_version());
else
    fprintf('validate_package: %d of %d CHECKS FAILED\n', v.n_fail, v.n_checks);
end
end

% ------------------------------------------------------------------ helpers
function v = check(v, ok, label)
v.n_checks = v.n_checks + 1;
if ok
    v.lines{end+1} = sprintf('  ok    %s', label); %#ok<AGROW>
else
    v.n_fail = v.n_fail + 1;
    v.ok = false;
    v.failures{end+1} = label; %#ok<AGROW>
    v.lines{end+1} = sprintf('  FAIL  %s', label); %#ok<AGROW>
end
end

function v = note(v, label)
v.lines{end+1} = sprintf('  --    %s', label); %#ok<AGROW>
end

function bn = blk_of_gen(gen)
% Map a generator name to its block name through the mode-A spec.
sp = gpr_pipeline_spec(config_mode_A_uav_shallow());
bn = '';
for sec = {'rf', 'dsp'}
    S = sp.(sec{1});
    for j = 1:numel(S.blocks)
        if strcmp(S.blocks{j}.gen, gen)
            bn = S.blocks{j}.name;
            return;
        end
    end
end
end
