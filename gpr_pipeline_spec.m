function spec = gpr_pipeline_spec(cfg)
% GPR_PIPELINE_SPEC  Single source of truth for the GPR processing chain.
%
%   spec = GPR_PIPELINE_SPEC(cfg) describes the whole algorithm as a
%   directed graph of blocks:
%
%       spec.params        {name, value} rows -> base-workspace parameters
%       spec.rf.consts     constant blocks inside RF_Front_End
%       spec.rf.blocks     algorithm blocks B01..B07
%       spec.rf.outports   subsystem output ports and their sources
%       spec.dsp.consts    constant blocks inside Digital_Processing
%       spec.dsp.blocks    algorithm blocks B08..B14
%       spec.dsp.inports   subsystem input ports
%       spec.top           top-level wiring and report sinks
%
%   Every algorithm block entry has
%       .name    block name (also used for its TPxx test point)
%       .gen     name of the function in code_*.m that returns its script
%       .pos     Simulink block position [x1 y1 x2 y2]
%       .inputs  cell array of source references, in input-port order
%
%   A source reference is 'BLOCK' (output port 1) or 'BLOCK:N' (output port
%   N). Constant blocks are referenced by their own name.
%
%   Both build_realistic_model.m (Simulink) and simulate_pipeline_reference.m
%   (plain MATLAB/Octave, no Simulink) build their pipeline from this one
%   file, so the model and the reference simulation cannot drift apart.
%
%   Test points are numbered in execution order: TP01..TP14.

if nargin < 1 || isempty(cfg)
    cfg = config_mode_B_ground_deep();
end

% ------------------------------------------------------------------ params
% One {name, value} PAIR PER ROW (semicolon separated). The previous version
% of this package used commas, which builds a 1-by-2N row cell, so the loop
% in create_all_params only ever created the first parameter.
p = {};
p = add_row(p, 'GPR_F_START',       cfg.f_start);
p = add_row(p, 'GPR_F_STOP',        cfg.f_stop);
p = add_row(p, 'GPR_UAV_ALTITUDE',  cfg.uav_altitude);
p = add_row(p, 'GPR_SCAN_LENGTH',   cfg.scan_length);
p = add_row(p, 'GPR_SOIL_MOISTURE', cfg.soil_moisture);
p = add_row(p, 'GPR_SOIL_SIGMA',    cfg.soil_conductivity);
p = add_row(p, 'GPR_PA_GAIN_DB',    cfg.pa_gain_dB);
p = add_row(p, 'GPR_LNA_GAIN_DB',   cfg.lna_gain_dB);
p = add_row(p, 'GPR_NF_DB',         cfg.system_nf_dB);
p = add_row(p, 'GPR_ADC_ENOB',      cfg.adc_enob);
p = add_row(p, 'GPR_N_AVG',         cfg.n_averages);
p = add_row(p, 'GPR_KAISER_BETA',   cfg.kaiser_beta);
p = add_row(p, 'GPR_DEPTH_MIN',     cfg.depth_min);
p = add_row(p, 'GPR_DEPTH_MAX',     cfg.depth_max);
p = add_row(p, 'GPR_PFA',           cfg.cfar_pfa);
p = add_row(p, 'GPR_TGT_DEPTH',     cfg.target_depths(:).');
p = add_row(p, 'GPR_TGT_X',         cfg.target_x(:).');
p = add_row(p, 'GPR_TGT_RCS',       cfg.target_rcs(:).');
p = add_row(p, 'GPR_COUPLING_DB',   cfg.antenna_coupling_dB);
p = add_row(p, 'GPR_BG_MODES',      cfg.bg_remove_modes);
p = add_row(p, 'GPR_TONE_BW',       cfg.tone_bandwidth);
spec.params = p;

% ------------------------------------------------------------- RF front end
c = {};
c = add_const(c, 'P_ALT',      'GPR_UAV_ALTITUDE', [ 20   30   90   55]);
c = add_const(c, 'P_SCAN',     'GPR_SCAN_LENGTH',  [ 20   70   90   95]);
c = add_const(c, 'P_FSTART',   'GPR_F_START',      [ 20  120   90  145]);
c = add_const(c, 'P_FSTOP',    'GPR_F_STOP',       [ 20  160   90  185]);
c = add_const(c, 'P_PA',       'GPR_PA_GAIN_DB',   [ 20  230   90  255]);
c = add_const(c, 'P_MOIST',    'GPR_SOIL_MOISTURE',[ 20  500   90  525]);
c = add_const(c, 'P_SIGMA',    'GPR_SOIL_SIGMA',   [ 20  540   90  565]);
c = add_const(c, 'P_TGT_D',    'GPR_TGT_DEPTH',    [ 20  580   90  605]);
c = add_const(c, 'P_TGT_X',    'GPR_TGT_X',        [ 20  620   90  645]);
c = add_const(c, 'P_TGT_RCS',  'GPR_TGT_RCS',      [ 20  660   90  685]);
c = add_const(c, 'P_COUPLING', 'GPR_COUPLING_DB',  [ 20  760   90  785]);
c = add_const(c, 'P_LNA',      'GPR_LNA_GAIN_DB',  [ 20  900   90  925]);
c = add_const(c, 'P_NF',       'GPR_NF_DB',        [ 20  940   90  965]);
c = add_const(c, 'P_TONE_BW',  'GPR_TONE_BW',      [ 20  980   90 1005]);
c = add_const(c, 'P_ENOB',     'GPR_ADC_ENOB',     [ 20 1040   90 1065]);
spec.rf.consts = c;

b = {};
b = add_blk(b, 'B01_Environment', 'code_environment', [220   30  420   80], ...
    {'P_ALT', 'P_SCAN'});
b = add_blk(b, 'B02_Waveform', 'code_waveform', [500   30  700   80], ...
    {'P_FSTART', 'P_FSTOP'});
b = add_blk(b, 'B03_TX_Chain', 'code_tx', [780   30  980   80], ...
    {'B02_Waveform', 'P_PA'});
b = add_blk(b, 'B04_Channel', 'code_channel', [500  180  760  240], ...
    {'B01_Environment', 'B03_TX_Chain', 'P_MOIST', 'P_SIGMA', 'P_TGT_D', ...
     'P_TGT_X', 'P_TGT_RCS', 'P_FSTART', 'P_FSTOP'});
b = add_blk(b, 'B05_Coupling', 'code_coupling', [820  180 1040  240], ...
    {'B03_TX_Chain', 'P_COUPLING', 'B04_Channel'});
b = add_blk(b, 'B06_RX', 'code_rx', [500  340  700  390], ...
    {'B05_Coupling', 'P_LNA', 'P_NF', 'P_TONE_BW'});
% Input 3 of the ADC is output port 2 of B06_RX (input-referred noise level).
b = add_blk(b, 'B07_ADC', 'code_adc', [780  340  980  390], ...
    {'B06_RX:1', 'P_ENOB', 'B06_RX:2'});
spec.rf.blocks = b;

spec.rf.outports = { ...
    struct('name', 'RF_Out',       'pos', [1200 300 1230 320], 'source', 'B07_ADC:1'), ...
    struct('name', 'RF_Noise_Out', 'pos', [1200 380 1230 400], 'source', 'B07_ADC:2')};

% -------------------------------------------------------- digital processing
c = {};
c = add_const(c, 'P_NAVG',   'GPR_N_AVG',       [ 20   40   90   65]);
c = add_const(c, 'P_ALT',    'GPR_UAV_ALTITUDE',[ 20  440   90  465]);
c = add_const(c, 'P_KAISER', 'GPR_KAISER_BETA', [ 20   80   90  105]);
c = add_const(c, 'P_FSTART', 'GPR_F_START',     [ 20  120   90  145]);
c = add_const(c, 'P_FSTOP',  'GPR_F_STOP',      [ 20  160   90  185]);
c = add_const(c, 'P_MOIST',  'GPR_SOIL_MOISTURE',[20  200   90  225]);
c = add_const(c, 'P_SIGMA',  'GPR_SOIL_SIGMA',  [ 20  240   90  265]);
c = add_const(c, 'P_NBG',    'GPR_BG_MODES',    [ 20  280   90  305]);
c = add_const(c, 'P_DMIN',   'GPR_DEPTH_MIN',   [ 20  520   90  545]);
c = add_const(c, 'P_DMAX',   'GPR_DEPTH_MAX',   [ 20  560   90  585]);
c = add_const(c, 'P_PFA',    'GPR_PFA',         [ 20  600   90  625]);
c = add_const(c, 'P_SCAN',   'GPR_SCAN_LENGTH', [ 20  640   90  665]);
spec.dsp.consts = c;

b = {};
b = add_blk(b, 'B08_Averaging', 'code_averaging', [180  280  380  330], ...
    {'RF_In', 'RF_Noise_In', 'P_NAVG'});
b = add_blk(b, 'B09_Calibration', 'code_calibration', [430  280  630  330], ...
    {'B08_Averaging', 'P_FSTART', 'P_FSTOP'});
b = add_blk(b, 'B10_RangeProc', 'code_rangeproc', [680  280  880  330], ...
    {'B09_Calibration', 'P_KAISER'});
b = add_blk(b, 'B11_Background', 'code_background', [930  280 1130  330], ...
    {'B10_RangeProc', 'P_NBG'});
b = add_blk(b, 'B12_Migration', 'code_migration', [430  420  690  480], ...
    {'B11_Background', 'P_FSTART', 'P_FSTOP', 'P_MOIST', 'P_SIGMA', 'P_DMAX', ...
     'P_SCAN', 'P_ALT'});
b = add_blk(b, 'B13_Detection', 'code_detection', [740  420 1000  480], ...
    {'B12_Migration', 'P_DMIN', 'P_DMAX', 'P_PFA', 'P_FSTART', 'P_FSTOP', 'P_MOIST', ...
     'P_SIGMA'});
b = add_blk(b, 'B14_Report', 'code_report', [1040  420 1240  480], ...
    {'B13_Detection', 'B12_Migration', 'P_FSTART', 'P_FSTOP', 'P_MOIST', ...
     'P_SIGMA', 'P_SCAN'});
spec.dsp.blocks = b;

spec.dsp.inports = { ...
    struct('name', 'RF_In',       'pos', [20 300 50 320], 'source', 'B07_ADC:1'), ...
    struct('name', 'RF_Noise_In', 'pos', [20 380 50 400], 'source', 'B07_ADC:2')};
spec.dsp.outports = { ...
    struct('name', 'Report_Out', 'pos', [1300 420 1330 440], 'source', 'B14_Report:1')};

% ---------------------------------------------------------------- top level
spec.top.rf_pos   = [ 80 120  390 420];
spec.top.dsp_pos  = [500 120  820 420];
spec.top.links = { ...
    struct('src', 'RF_Front_End:1', 'dst', 'Digital_Processing:1'), ...
    struct('src', 'RF_Front_End:2', 'dst', 'Digital_Processing:2'), ...
    struct('src', 'Digital_Processing:1', 'dst', 'Final_Report_Log:1'), ...
    struct('src', 'Digital_Processing:1', 'dst', 'Final_Report_Display:1')};
spec.top.log_pos  = [900 230 1040 260];
spec.top.disp_pos = [900 300 1040 330];

% Test point numbering: RF = TP01..TP07, DSP = TP08..TP14.
spec.rf.first_tp  = 1;
spec.dsp.first_tp = 8;

spec.model_name = 'GPR_Realistic';
end

% ------------------------------------------------------------- local helpers
function p = add_row(p, name, value)
p(end+1, :) = {name, value}; %#ok<AGROW>
end

function c = add_const(c, name, param, pos)
c(end+1) = {struct('name', name, 'param', param, 'pos', pos)}; %#ok<AGROW>
end

function b = add_blk(b, name, gen, pos, inputs)
b(end+1) = {struct('name', name, 'gen', gen, 'pos', pos, 'inputs', {inputs})}; %#ok<AGROW>
end
