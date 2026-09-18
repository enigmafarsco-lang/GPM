function off = gpr_path_sanity()
% GPR_PATH_SANITY  Detect a second GPM installation shadowing this one.
%
%   off = GPR_PATH_SANITY() returns one line per core function that does NOT
%   resolve into the folder of this file.  A stale copy of this package
%   earlier on the MATLAB path (an old unzip, a saved path, a support
%   folder) makes MATLAB mix two versions of the pipeline, which produces
%   errors that cannot be reproduced from the sources you are looking at.
%   Fix with:  restoredefaultpath; rehash toolboxcache; addpath(<this dir>);

here = fileparts(mfilename('fullpath'));
names = {'gpr_pipeline_spec', 'gpr_check_config', 'gpr_axes', ...
    'gpr_extract_results', 'gpr_compile_script', 'gpr_parse_script', ...
    'run_pipeline_reference', 'build_realistic_model', ...
    'gpr_block_io_sizes', 'check_physics_feasibility', ...
    'gpr_realistic_main', 'soil_permittivity', ...
    'code_environment', 'code_waveform', 'code_tx', 'code_channel', ...
    'code_coupling', 'code_rx', 'code_adc', 'code_averaging', ...
    'code_calibration', 'code_rangeproc', 'code_background', ...
    'code_migration', 'code_detection', 'code_report'};
off = {};
for k = 1:numel(names)
    w = which(names{k});
    if isempty(w)
        off{end+1} = sprintf('%s: NOT ON PATH', names{k}); %#ok<AGROW>
    elseif ~strcmp(fileparts(w), here)
        off{end+1} = sprintf('%s: resolves to %s', names{k}, w); %#ok<AGROW>
    end
end
end
