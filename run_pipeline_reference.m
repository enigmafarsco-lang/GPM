function res = run_pipeline_reference(cfg, varargin)
% RUN_PIPELINE_REFERENCE  Run the complete GPR chain WITHOUT Simulink.
%
%   res = RUN_PIPELINE_REFERENCE(cfg) executes exactly the code that
%   build_realistic_model.m pastes into the Simulink MATLAB Function blocks:
%   each code_*.m generator is called, its script is written to a temporary
%   function file (gpr_compile_script.m) and the blocks are then run in the
%   order and with the wiring given by gpr_pipeline_spec.m.
%
%   That makes this function the reference implementation of the model: it
%   needs only base MATLAB (it also runs in Octave), it produces the same
%   numbers as the model, and it is what test_gpr_package.m verifies.
%
%   Options (name/value):
%       'workdir'  folder for the generated block functions
%                  (default: tempdir/gpr_reference/<mode>)
%       'seed'     RNG seed for reproducible noise (default 1)
%       'verbose'  print per-block timing (default true)
%
%   res.tp      test-point values; field names are identical to the Simulink
%               To Workspace variable names (tp01_b01_environment, ...)
%   res.report  the 6x1 vector produced by B14_Report
%   plus the convenience fields added by gpr_extract_results.

if nargin < 1 || isempty(cfg)
    cfg = config_mode_B_ground_deep();
end
cfg = gpr_check_config(cfg, cfg.mode);

p = inputParser();
addParameter(p, 'workdir', '');
addParameter(p, 'seed', 1);
addParameter(p, 'verbose', true);
parse(p, varargin{:});
opt = p.Results;

off = gpr_path_sanity();
if ~isempty(off)
    error('GPR:pathShadow', ...
        ['Mixed GPM installations on the MATLAB path - MATLAB would run a\n' ...
         'blend of two versions of this package:\n%s\n' ...
         'Fix with:  restoredefaultpath; rehash toolboxcache; addpath(<this folder>);'], ...
        strjoin(off, sprintf('\n')));
end

if isempty(opt.workdir)
    safe = regexprep(cfg.mode, '[^A-Za-z0-9_]', '_');
    opt.workdir = fullfile(tempdir, 'gpr_reference', safe);
end
if ~exist(opt.workdir, 'dir')
    mkdir(opt.workdir); %#ok<CTCH>
end

if opt.seed > 0
    rng(opt.seed);
end

spec = gpr_pipeline_spec(cfg);

% Base-workspace parameters, exactly as create_all_params would install them.
vals = struct();
for k = 1:size(spec.params, 1)
    vals.(spec.params{k,1}) = {spec.params{k,2}};
end

tp = struct();
blk_idx = 0;

fprintf('Reference pipeline: %s\n', cfg.mode);
[vals, tp, blk_idx] = run_section(spec.rf.consts, spec.rf.blocks, cfg, opt.workdir, ...
    vals, tp, blk_idx, opt.verbose);

% The digital chain is fed through the subsystem input ports.
for k = 1:numel(spec.dsp.inports)
    ip = spec.dsp.inports{k};
    vals.(ip.name) = resolve(ip.source, vals, 'input port');
end
[vals, tp, blk_idx] = run_section(spec.dsp.consts, spec.dsp.blocks, cfg, opt.workdir, ...
    vals, tp, blk_idx, opt.verbose);

res = gpr_extract_results(tp, cfg, []);
res.spec = spec;
res.vals = vals;
res.workdir = opt.workdir;
res.source = 'reference';
end

% ------------------------------------------------------------------ helpers
function [vals, tp, blk_idx] = run_section(consts, blocks, cfg, workdir, vals, tp, blk_idx, verbose)
for k = 1:numel(consts)
    cc = consts{k};
    if ~isfield(vals, cc.param)
        error('gpr:pipeline:missingParam', ...
            'Constant block %s refers to unknown parameter %s.', cc.name, cc.param);
    end
    vals.(cc.name) = vals.(cc.param);
end
for k = 1:numel(blocks)
    blk = blocks{k};
    script = feval(blk.gen, cfg);
    fname = ['gpr_ref_' lower(regexprep(blk.name, '[^A-Za-z0-9_]', '_'))];
    [fh, info] = gpr_compile_script(script, fname, workdir);
    if info.nin ~= numel(blk.inputs)
        error('gpr:pipeline:portMismatch', ...
            ['%s declares %d input port(s) but the spec wires %d. ' ...
             'Fix gpr_pipeline_spec.m or %s.m.'], ...
            blk.name, info.nin, numel(blk.inputs), blk.gen);
    end
    args = cell(1, numel(blk.inputs));
    for q = 1:numel(blk.inputs)
        got = resolve(blk.inputs{q}, vals, blk.name);
        args{q} = got{1};
    end
    t0 = tic();
    out = cell(1, info.nout);
    [out{:}] = fh(args{:});
    el = toc(t0);
    vals.(blk.name) = out;

    blk_idx = blk_idx + 1;
    base = sprintf('TP%02d_%s', blk_idx, blk.name);
    for q = 1:numel(out)
        if q == 1
            varname = lower(base);
        else
            varname = lower(sprintf('%s_o%d', base, q));
        end
        tp.(varname) = out{q};
    end
    if verbose
        fprintf('  %-18s %7.2f s  %s\n', blk.name, el, describe(out));
    end
end
end

function got = resolve(ref, vals, who)
% Resolve 'BLOCK' or 'BLOCK:port' into a one-element cell array.
tok = strsplit(ref, ':');
name = tok{1};
port = 1;
if numel(tok) > 1
    port = str2double(tok{2});
end
if ~isfield(vals, name)
    error('gpr:pipeline:unresolved', ...
        '%s needs ''%s'', which is not defined at that point in the spec.', who, ref);
end
v = vals.(name);
if ~iscell(v)
    v = {v};
end
if port < 1 || port > numel(v)
    error('gpr:pipeline:portRange', ...
        '%s asks for output port %d of %s, which only has %d output(s).', ...
        who, port, name, numel(v));
end
got = v(port);
end

function s = describe(out)
parts = cell(1, numel(out));
for k = 1:numel(out)
    v = out{k};
    if isnumeric(v)
        cx = '';
        if ~isreal(v), cx = ' complex'; end
        parts{k} = sprintf('%dx%d%s', size(v,1), size(v,2), cx);
    else
        parts{k} = class(v);
    end
end
s = strjoin(parts, ', ');
end
