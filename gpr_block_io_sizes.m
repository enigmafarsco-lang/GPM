function io = gpr_block_io_sizes(cfg)
% GPR_BLOCK_IO_SIZES  Analytic port sizes/types of every algorithm block.
%
%   io = GPR_BLOCK_IO_SIZES(cfg) returns a struct with one field per block
%   (B01_Environment ... B14_Report); each field is a struct with
%       .in_sz   cell of [rows cols] per input port, in spec wiring order
%       .in_cx   cell of 'Real'/'Complex' per input port
%       .out_sz  cell of [rows cols] per output port
%       .out_cx  cell of 'Real'/'Complex' per output port
%
%   Why this file exists: a MATLAB Function block whose output size has to
%   be inferred through a chain of other MATLAB Function blocks can fail
%   Simulink's size propagation ("Simulink does not have enough information
%   to determine output sizes for this block").  Every size in this chain
%   is known analytically from the configuration, so build_realistic_model
%   stamps them onto the Stateflow chart data explicitly.  Input sizes are
%   derived from the spec wiring, so this table cannot disagree with
%   gpr_pipeline_spec.m.

if nargin < 1 || isempty(cfg)
    cfg = config_mode_B_ground_deep();
end
cfg = gpr_check_config(cfg, cfg.mode);

NT = cfg.n_tones; NP = cfg.n_positions; NI = cfg.n_ifft; NTGT = cfg.n_targets;

osz = containers.Map(); ocx = containers.Map();
osz('B01_Environment') = {[3 NP]};              ocx('B01_Environment') = {'Real'};
osz('B02_Waveform')    = {[NT 1]};              ocx('B02_Waveform')    = {'Complex'};
osz('B03_TX_Chain')    = {[NT 1]};              ocx('B03_TX_Chain')    = {'Complex'};
osz('B04_Channel')     = {[NT NP]};             ocx('B04_Channel')     = {'Complex'};
osz('B05_Coupling')    = {[NT NP]};             ocx('B05_Coupling')    = {'Complex'};
osz('B06_RX')          = {[NT NP], [1 1]};      ocx('B06_RX')          = {'Complex', 'Real'};
osz('B07_ADC')         = {[NT NP], [NP 1]};     ocx('B07_ADC')         = {'Complex', 'Real'};
osz('B08_Averaging')   = {[NT NP]};             ocx('B08_Averaging')   = {'Complex'};
osz('B09_Calibration') = {[NT NP]};             ocx('B09_Calibration') = {'Complex'};
osz('B10_RangeProc')   = {[NI NP]};             ocx('B10_RangeProc')   = {'Complex'};
osz('B11_Background')  = {[NI NP]};             ocx('B11_Background')  = {'Complex'};
osz('B12_Migration')   = {[NI NP]};             ocx('B12_Migration')   = {'Real'};
osz('B13_Detection')   = {[NI NP]};             ocx('B13_Detection')   = {'Real'};
osz('B14_Report')      = {[6 1]};               ocx('B14_Report')      = {'Real'};

% Constant blocks: everything is a scalar except the target vectors.
vec_param = containers.Map({'GPR_TGT_DEPTH', 'GPR_TGT_X', 'GPR_TGT_RCS'}, ...
    {[1 NTGT], [1 NTGT], [1 NTGT]});
% Subsystem input ports of Digital_Processing.
ip_sz = containers.Map({'RF_In', 'RF_Noise_In'}, {[NT NP], [NP 1]});
ip_cx = containers.Map({'RF_In', 'RF_Noise_In'}, {'Complex', 'Real'});

spec = gpr_pipeline_spec(cfg);
io = struct();
for sec = {'rf', 'dsp'}
    S = spec.(sec{1});
    cparam = containers.Map();
    for k = 1:numel(S.consts)
        cparam(S.consts{k}.name) = S.consts{k}.param;
    end
    for k = 1:numel(S.blocks)
        b = S.blocks{k};
        n = numel(b.inputs);
        insz = cell(1, n);
        incx = cell(1, n);
        for q = 1:n
            ref = b.inputs{q};
            [bn, prt] = strtok(ref, ':');
            prt = str2double(strrep(prt, ':', ''));
            if isempty(prt) || isnan(prt), prt = 1; end
            if isKey(osz, bn)
                % MATLAB forbids chaining brace indexing onto a containers.Map
                % call (osz(bn){prt}); Octave accepts it.  Use a temporary.
                szs = osz(bn); cxs = ocx(bn);
                insz{q} = szs{prt};
                incx{q} = cxs{prt};
            elseif isKey(ip_sz, bn)
                insz{q} = ip_sz(bn);
                incx{q} = ip_cx(bn);
            elseif isKey(cparam, bn) && isKey(vec_param, cparam(bn))
                insz{q} = vec_param(cparam(bn));
                incx{q} = 'Real';
            else
                insz{q} = [1 1];
                incx{q} = 'Real';
            end
        end
        % The analytic table is only the fallback; the authoritative sizes
        % and complexities are PROBED: run the compiled block once on typed
        % dummy inputs and stamp exactly what it produces.  A hand table can
        % disagree with the script (it did, twice); a probe cannot.
        [psz, pcx] = probe_block(b, cfg, insz, incx, numel(osz(b.name)));
        if isempty(psz)
            psz = osz(b.name);
            pcx = ocx(b.name);
        end
        % wrap the cells: struct() would otherwise distribute them
        io.(b.name) = struct('in_sz', {insz}, 'in_cx', {incx}, ...
            'out_sz', {psz}, 'out_cx', {pcx}, ...
            'table_sz', {osz(b.name)}, 'table_cx', {ocx(b.name)});
    end
end
end

function [psz, pcx] = probe_block(b, cfg, insz, incx, nout)
% Run the generated block script once on ones() inputs of the wired sizes
% and record the true size/complexity of every output.  An empty return
% means the probe failed and the caller keeps the analytic table.
psz = {};
pcx = {};
try
    wd = fullfile(tempdir, 'gpr_io_probe');
    if ~exist(wd, 'dir'), mkdir(wd); end
    nm = ['iopr_' lower(b.name)];
    fh = gpr_compile_script(feval(b.gen, cfg), nm, wd);
    args = cell(1, numel(insz));
    for q = 1:numel(insz)
        if strcmp(incx{q}, 'Complex')
            % isreal() tests VALUES, not types: the dummy must carry a
            % non-zero imaginary part AND vary across traces, because
            % trace-invariant components are exactly what B11 removes by
            % design (a constant dummy would probe a complex path as real)
            r = insz{q}(1); c = insz{q}(2);
            args{q} = ones(r, c) + 0.5i ...
                + 0.01*repmat(mod(1:c, 3), r, 1) ...
                + 0.01i*repmat(mod(1:c, 2), r, 1);
        else
            args{q} = ones(insz{q}(1), insz{q}(2));
        end
    end
    outs = cell(1, nout);
    [outs{1:nout}] = fh(args{:});
    for q = 1:nout
        psz{q} = size(outs{q}); %#ok<AGROW>
        if isreal(outs{q})
            pcx{q} = 'Real'; %#ok<AGROW>
        else
            pcx{q} = 'Complex'; %#ok<AGROW>
        end
    end
catch
    psz = {};
    pcx = {};
end
end
