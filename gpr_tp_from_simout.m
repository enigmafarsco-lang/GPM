function tp = gpr_tp_from_simout(out, cfg)
% GPR_TP_FROM_SIMOUT  Collect the 14 test points after a Simulink run.
%
%   tp = GPR_TP_FROM_SIMOUT(out, cfg) returns the struct that
%   gpr_extract_results expects, with one field per logged test point
%   (tp01_b01_environment ... tp14_b14_report, plus tpNN_..._o2 for the
%   blocks that have a second output port).
%
%   OUT may be
%       []                          - read the variables from the base
%                                     workspace (build_realistic_model sets
%                                     ReturnWorkspaceOutputs = off)
%       Simulink.SimulationOutput   - read them from the simulation output
%       struct                      - read them from the struct fields
%
%   Values may be plain arrays, timeseries or "Structure With Time" logs;
%   gpr_extract_results normalises all three.  The names are derived from
%   gpr_pipeline_spec(cfg), i.e. exactly the names the model's To Workspace
%   blocks and run_pipeline_reference use.

if nargin < 2 || isempty(cfg)
    cfg = config_mode_B_ground_deep();
end
cfg = gpr_check_config(cfg, cfg.mode);
spec = gpr_pipeline_spec(cfg);

names = tp_names(spec, cfg);
tp = struct();
missing = {};

for k = 1:numel(names)
    nm = names{k};
    v = fetch_one(out, nm);
    if isempty(v)
        missing{end+1} = nm; %#ok<AGROW>
    else
        tp.(nm) = v;
    end
end

if ~isempty(missing)
    warning('GPR:simout:missing', ...
        ['%d test point(s) were not logged: %s.\n' ...
         'Simulate the model built by build_realistic_model.m first, or use ' ...
         'run_pipeline_reference(cfg), which needs no Simulink.'], ...
        numel(missing), strjoin(missing, ', '));
end
end

% ------------------------------------------------------------------ helpers
function names = tp_names(spec, cfg)
names = {};
sections = {spec.rf.blocks, spec.dsp.blocks};
tpnums = [spec.rf.first_tp, spec.dsp.first_tp];
for s = 1:2
    blocks = sections{s};
    tp = tpnums(s);
    for k = 1:numel(blocks)
        b = blocks{k};
        base = sprintf('TP%02d_%s', tp, b.name);
        names{end+1} = lower(base); %#ok<AGROW>
        info = gpr_parse_script(feval(b.gen, cfg));
        for q = 2:info.nout
            names{end+1} = lower(sprintf('%s_o%d', base, q)); %#ok<AGROW>
        end
        tp = tp + 1;
    end
end
end

function v = fetch_one(out, nm)
v = [];
if ~isempty(out)
    if isa(out, 'Simulink.SimulationOutput')
        try
            v = out.get(nm);
        catch
            v = [];
        end
    elseif isstruct(out) && isfield(out, nm)
        v = out.(nm);
    end
end
if isempty(v)
    if evalin('base', sprintf('exist(''%s'', ''var'')', nm))
        v = evalin('base', nm);
    end
end
end
