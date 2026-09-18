function res = gpr_realistic_main(varargin)
% GPR_REALISTIC_MAIN  Run the GPR digital twin end to end.
%
%   res = GPR_REALISTIC_MAIN()                  mode B, Simulink if present
%   res = GPR_REALISTIC_MAIN('mode', 'A')       mode A
%   res = GPR_REALISTIC_MAIN('engine','reference')
%                                               no Simulink: execute exactly
%                                               the same generated block
%                                               scripts in MATLAB/Octave
%
%   Name/value options
%       'mode'      'A' | 'B' | a config struct     (default 'B')
%       'engine'    'auto' | 'simulink' | 'reference'   (default 'auto')
%       'seed'      RNG seed for the noise (default 1, 0 = random)
%       'build'     rebuild the .slx even if it exists (default true)
%       'gui'       open gpr_realistic_gui when done  (default false)
%       'save'      save results to gpr_results_<mode>.mat (default true)
%       'verbose'   print the feasibility report and progress (default true)
%
%   Steps
%       1. load the configuration and validate it (gpr_check_config)
%       2. audit the physics (check_physics_feasibility)
%       3. build the model (build_realistic_model) or use the reference runner
%       4. simulate / execute
%       5. extract the results (gpr_extract_results) and compare with truth
%
%   res is the struct documented in gpr_extract_results, plus
%       .engine       'simulink' or 'reference'
%       .truth        the configured target list
%       .match        per-target detection error (NaN when missed)
%       .model        the model name (Simulink engine only)

p = inputParser();
addParameter(p, 'mode', 'B');
addParameter(p, 'engine', 'auto');
addParameter(p, 'seed', 1);
addParameter(p, 'build', true);
addParameter(p, 'gui', false);
addParameter(p, 'save', true);
addParameter(p, 'verbose', true);
parse(p, varargin{:});
opt = p.Results;

here = fileparts(mfilename('fullpath'));
if ~isempty(here) && isempty(strfind(path, here)) %#ok<STREMP>
    addpath(here);
end

off = gpr_path_sanity();
if ~isempty(off)
    error('GPR:pathShadow', ...
        ['Mixed GPM installations on the MATLAB path - MATLAB would run a\n' ...
         'blend of two versions of this package:\n%s\n' ...
         'Fix with:  restoredefaultpath; rehash toolboxcache; addpath(<this folder>);'], ...
        strjoin(off, sprintf('\n')));
end

% ------------------------------------------------------------- configuration
if isstruct(opt.mode)
    cfg = opt.mode;
else
    switch upper(char(opt.mode))
        case 'A', cfg = config_mode_A_uav_shallow();
        case 'B', cfg = config_mode_B_ground_deep();
        otherwise
            error('GPR:main:mode', 'mode must be ''A'', ''B'' or a config struct.');
    end
end
cfg = gpr_check_config(cfg, cfg.mode);

if opt.verbose
    feas = check_physics_feasibility(cfg, 'verbose', true);
    fprintf('\n');
else
    feas = check_physics_feasibility(cfg, 'verbose', false);
end
if ~feas.ok
    warning('GPR:main:infeasible', ...
        'The configuration failed the physics audit; results will be poor.');
end

% ------------------------------------------------------------------ engine
engine = lower(char(opt.engine));
have_simulink = ~isempty(which('sim')) && license('test', 'Simulink'); %#ok<LICTST>
switch engine
    case 'auto'
        if have_simulink, engine = 'simulink'; else, engine = 'reference'; end
    case {'simulink','reference'}
        % as requested
    otherwise
        error('GPR:main:engine', 'engine must be auto, simulink or reference.');
end
if strcmp(engine, 'simulink') && ~have_simulink
    warning('GPR:main:noSimulink', ...
        'Simulink is not available - falling back to the reference engine.');
    engine = 'reference';
end

% --------------------------------------------------------------------- run
t0 = tic();
if strcmp(engine, 'reference')
    res = run_pipeline_reference(cfg, 'seed', opt.seed, 'verbose', opt.verbose);
    res.model = '';
else
    model = build_realistic_model(cfg, 'verbose', opt.verbose, ...
        'overwrite', logical(opt.build));
    if opt.seed > 0
        rng(opt.seed);
    end
    out = sim(model);
    tp = gpr_tp_from_simout(out, cfg);
    res = gpr_extract_results(tp, cfg, feas);
    res.model = model;
end
res.engine = engine;
res.elapsed_s = toc(t0);

% ------------------------------------------------------------ score vs truth
res.truth = struct('x', cfg.target_x, 'depth', cfg.target_depths, ...
    'rcs', cfg.target_rcs);
[res.match, res.unused_det] = score_against_truth(res, cfg);

if opt.verbose
    print_summary(res, cfg);
end
if opt.save
    file = sprintf('gpr_results_%s.mat', regexprep(cfg.mode, '[^A-Za-z0-9_]', '_'));
    save(file, 'res', 'cfg');
    if opt.verbose
        fprintf('saved %s\n', file);
    end
end
if opt.gui
    gpr_realistic_gui(res);
end
end

% ------------------------------------------------------------------ helpers
function [m, unused] = score_against_truth(res, cfg)
% Nearest detection for every configured target, within the system
% resolution.  Missed targets get NaN.
m = nan(cfg.n_targets, 3);
unused = false(1, res.n_detections);
used = unused;
tol_d = max(2*cfg.depth_resolution, 0.05);
tol_x = max(3*cfg.dx_trace, 0.3);
for t = 1:cfg.n_targets
    best = 0; bd = inf;
    for k = 1:res.n_detections
        d = res.detections(k);
        dd = abs(d.depth - cfg.target_depths(t));
        dxx = abs(d.x - cfg.target_x(t));
        if ~used(k) && dd <= tol_d && dxx <= tol_x && dd < bd
            bd = dd; best = k;
        end
    end
    if best > 0
        used(best) = true;
        d = res.detections(best);
        m(t, :) = [d.x - cfg.target_x(t), d.depth - cfg.target_depths(t), d.amp];
    end
end
unused = ~used;   % true for a detection that matches no configured target
end

function print_summary(res, cfg)
fprintf('\n=== results (%s engine, %.1f s) ===\n', res.engine, res.elapsed_s);
fprintf('report vector: [%s]\n', sprintf('%.6g ', res.report));
fprintf(' tgt |   true x    true d |    det x     det d |    dx        dd    \n');
for t = 1:cfg.n_targets
    if isnan(res.match(t, 1))
        fprintf('  %d  | %8.2f %9.2f |   MISSED\n', t, cfg.target_x(t), ...
            cfg.target_depths(t));
    else
        fprintf('  %d  | %8.2f %9.2f | %8.2f %9.2f | %+7.3f  %+7.3f\n', t, ...
            cfg.target_x(t), cfg.target_depths(t), ...
            cfg.target_x(t) + res.match(t,1), cfg.target_depths(t) + res.match(t,2), ...
            res.match(t,1), res.match(t,2));
    end
end
ndet = sum(~isnan(res.match(:, 1)));
nfalse = sum(res.unused_det);
fprintf('detected %d/%d targets, %d unmatched cluster(s), %d detection pixel(s)\n', ...
    ndet, cfg.n_targets, nfalse, res.n_pixels);
end
