function model = build_realistic_model(cfg, varargin)
% BUILD_REALISTIC_MODEL  Build the Simulink digital twin of the GPR chain.
%
%   model = BUILD_REALISTIC_MODEL(cfg) builds GPR_Realistic.slx from the
%   configuration struct returned by config_mode_A_uav_shallow() or
%   config_mode_B_ground_deep().  model = BUILD_REALISTIC_MODEL() uses mode B.
%
%   Name/value options
%       'model_name'  name of the model          (default 'GPR_Realistic')
%       'folder'      where to save the .slx     (default current folder)
%       'open'        open the model when done   (default false)
%       'overwrite'   replace an existing model  (default true)
%       'verbose'     print progress             (default true)
%
%   What gets built
%   ---------------
%       RF_Front_End        B01 Environment .. B07 ADC  (+ TP01..TP07)
%       Digital_Processing  B08 Averaging .. B14 Report (+ TP08..TP14)
%       top level           the two subsystems, Final_Report_Log (To
%                           Workspace) and Final_Report_Display
%
%   Everything - block list, port wiring, test-point numbering, base
%   workspace parameters - comes from gpr_pipeline_spec(cfg), which is the
%   same description run_pipeline_reference.m executes without Simulink.  The
%   model and the reference simulation therefore cannot drift apart, and
%   test_gpr_package.m can compare them block by block.
%
%   Each algorithm block is a MATLAB Function block whose Script is the text
%   returned by its code_*.m generator.  The scripts call two shared helpers
%   (soil_permittivity.m and gpr_range_window.m), so THIS FOLDER MUST BE ON
%   THE MATLAB PATH when the model is updated, simulated or code-generated;
%   the function adds it automatically.
%
%   Requirements: Simulink (any release since R2016b) and Stateflow for the
%   MATLAB Function blocks.  No other toolbox is used.

if nargin < 1 || isempty(cfg)
    cfg = config_mode_B_ground_deep();
end
cfg = gpr_check_config(cfg, cfg.mode);

p = inputParser();
addParameter(p, 'model_name', '', @(s) ischar(s) || isstring(s));
addParameter(p, 'folder', '', @(s) ischar(s) || isstring(s));
addParameter(p, 'open', false, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'overwrite', true, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'verbose', true, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
opt = p.Results;

spec = gpr_pipeline_spec(cfg);
if isempty(opt.model_name)
    opt.model_name = spec.model_name;
end
model = char(opt.model_name);
if isempty(opt.folder)
    opt.folder = pwd;
end

% --------------------------------------------------------------- the path
here = fileparts(mfilename('fullpath'));
if ~isempty(here) && isempty(strfind(path, here)) %#ok<STREMP>
    addpath(here);
end
require_simulink();

% ------------------------------------------- base workspace parameters
install_params(spec, opt.verbose);

% ------------------------------------------------------------- new model
if bdIsLoaded(model)
    if ~opt.overwrite
        error('GPR:build:modelOpen', 'Model %s is already loaded.', model);
    end
    close_system(model, 0);
end
slx = fullfile(opt.folder, [model '.slx']);
if exist(slx, 'file') && opt.overwrite
    delete(slx);
end

new_system(model);
set_param(model, ...
    'SolverType',   'Fixed-step', ...
    'Solver',       'FixedStepDiscrete', ...
    'StopTime',     num2str(stop_time(cfg)), ...
    'FixedStep',    num2str(gpr_cfg_value(cfg, 'sim_fixed_step', 1)), ...
    'SaveOutput',   'off', ...
    'SaveState',    'off', ...
    'SignalLogging','off', ...
    'ReturnWorkspaceOutputs', 'off');
if opt.verbose
    fprintf('Building %s (mode %s)...\n', model, cfg.mode);
end

% ------------------------------------------------------------ subsystems
build_subsystem(model, 'RF_Front_End', spec.rf, spec.rf.first_tp, cfg, opt.verbose);
build_subsystem(model, 'Digital_Processing', spec.dsp, spec.dsp.first_tp, cfg, opt.verbose);

% ------------------------------------------------------------- top level
set_param([model '/RF_Front_End'], 'Position', spec.top.rf_pos);
set_param([model '/Digital_Processing'], 'Position', spec.top.dsp_pos);

add_block('simulink/Sinks/To Workspace', [model '/Final_Report_Log'], ...
    'Position', spec.top.log_pos, 'VariableName', 'gpr_report', ...
    'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/Display', [model '/Final_Report_Display'], ...
    'Position', spec.top.disp_pos);

for k = 1:size(spec.top.links, 1)
    ln = spec.top.links{k};
    add_line(model, port_ref(ln.src), port_ref(ln.dst), 'autorouting', 'on');
end

% ------------------------------------------------- one single update only
% Updating earlier (the previous version did it while adding each block)
% makes Simulink analyse a MATLAB Function block before its Script has been
% replaced, which locks in the default "function y = fcn(u)" signature and
% then fails on every wiring check.
if opt.verbose
    fprintf('  updating diagram (compiles all 14 block scripts)...\n');
end
try
    set_param(model, 'SimulationCommand', 'update');
catch err
    save_system(model, slx);
    error('GPR:build:update', ...
        ['Updating the diagram failed; the unfinished model was saved to %s.\n' ...
         '  %s'], slx, err.message);
end

save_system(model, slx);
if opt.verbose
    fprintf('  saved %s\n', slx);
    fprintf('  %d blocks, %d test points, %d base parameters\n', ...
        numel(spec.rf.blocks) + numel(spec.dsp.blocks), 14, size(spec.params, 1));
end
if opt.open
    open_system(model);
end
end

% ================================================================= helpers
function require_simulink()
if ~license('test', 'Simulink') %#ok<LICTST>
    error('GPR:build:noSimulink', ...
        ['Simulink is not available.  Use run_pipeline_reference(cfg) instead: ' ...
         'it executes exactly the same generated block scripts without a model.']);
end
end

function t = stop_time(cfg)
% The whole chain is a one-shot batch computation (every block processes a
% complete profile matrix), so a single major time step is enough.  With a
% fixed step of 1 s a stop time of 0.5 s produces exactly one step at t = 0.
t = gpr_cfg_value(cfg, 'sim_stop_time', 0);
if ~(t > 0)
    t = 0.5*gpr_cfg_value(cfg, 'sim_fixed_step', 1);
end
end

function install_params(spec, verbose)
% One {name, value} pair per row - the previous version of this package built
% a 1x2N row cell with commas, so its loop only ever created the first one.
n = 0;
for k = 1:size(spec.params, 1)
    name = spec.params{k, 1};
    value = spec.params{k, 2};
    assignin('base', name, value);
    n = n + 1;
end
if verbose
    fprintf('  %d base workspace parameters installed\n', n);
end
end

function build_subsystem(model, sub, section, first_tp, cfg, verbose)
blk = [model '/' sub];
add_block('simulink/Ports & Subsystems/Subsystem', blk, 'Position', [0 0 100 100]);
% Remove the default In1/Out1 pair that ships with a new subsystem.
defaults = find_system(blk, 'SearchDepth', 1, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Type', 'block');
for k = 1:numel(defaults)
    if ~strcmp(defaults{k}, blk)
        delete_block(defaults{k});
    end
end

if verbose
    fprintf('  %s\n', sub);
end

% --- input ports (source references are resolved against the parent model)
if isfield(section, 'inports')
    for k = 1:numel(section.inports)
        ip = section.inports{k};
        add_block('simulink/Sources/In1', [blk '/' ip.name], ...
            'Position', ip.pos, 'Port', num2str(k));
    end
end

% --- constant blocks
for k = 1:numel(section.consts)
    cc = section.consts{k};
    add_block('simulink/Sources/Constant', [blk '/' cc.name], ...
        'Position', cc.pos, 'Value', cc.param);
end

% --- algorithm blocks, their scripts and their test points
tp = first_tp;
for k = 1:numel(section.blocks)
    b = section.blocks{k};
    path = [blk '/' b.name];
    add_block('simulink/User-Defined Functions/MATLAB Function', path, ...
        'Position', b.pos);
    script = feval(b.gen, cfg);
    info = gpr_parse_script(script);
    if info.nin ~= numel(b.inputs)
        error('GPR:build:portMismatch', ...
            ['%s declares %d input port(s) but the spec wires %d. ' ...
             'Fix gpr_pipeline_spec.m or %s.m.'], ...
            b.name, info.nin, numel(b.inputs), b.gen);
    end
    set_mlfcn_script(path, script);

    % one To Workspace per output port, named exactly like the reference run
    base = sprintf('TP%02d_%s', tp, b.name);
    ypos = b.pos(4);
    for q = 1:info.nout
        if q == 1
            varname = lower(base);
            tpname = base;
        else
            varname = lower(sprintf('%s_o%d', base, q));
            tpname = sprintf('%s_o%d', base, q);
        end
        tpos = [b.pos(1) + 20, ypos + 25 + 45*(q-1), ...
                b.pos(1) + 190, ypos + 50 + 45*(q-1)];
        add_block('simulink/Sinks/To Workspace', [blk '/' tpname], ...
            'Position', tpos, 'VariableName', varname, ...
            'SaveFormat', 'Timeseries');
        add_line(blk, sprintf('%s/%d', b.name, q), [tpname '/1'], ...
            'autorouting', 'on');
    end

    % data wiring
    for q = 1:numel(b.inputs)
        add_line(blk, port_ref(b.inputs{q}), sprintf('%s/%d', b.name, q), ...
            'autorouting', 'on');
    end
    tp = tp + 1;
end

% --- output ports
if isfield(section, 'outports')
    for k = 1:numel(section.outports)
        op = section.outports{k};
        add_block('simulink/Sinks/Out1', [blk '/' op.name], ...
            'Position', op.pos, 'Port', num2str(k));
        add_line(blk, port_ref(op.source), [op.name '/1'], 'autorouting', 'on');
    end
end
end

function ref = port_ref(src)
% 'BLOCK' -> 'BLOCK/1', 'BLOCK:N' -> 'BLOCK/N'
tok = strsplit(src, ':');
if numel(tok) > 1
    ref = [tok{1} '/' tok{2}];
else
    ref = [tok{1} '/1'];
end
end

function set_mlfcn_script(path, script)
% Replace the Script of a MATLAB Function block.  The Stateflow chart behind
% the block exists as soon as the block is added; it must NOT be updated
% before the script is in place (see the comment in build_realistic_model).
rt = sfroot();
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
if isempty(chart)
    % Older releases register the chart under the block name only.
    [~, nm] = fileparts(path);
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Name', nm);
    if numel(chart) > 1
        chart = chart(1);
    end
end
if isempty(chart)
    error('GPR:build:noChart', ...
        'Could not find the Stateflow chart behind MATLAB Function block %s.', path);
end
chart.Script = script;
end
