function res = gpr_extract_results(tp, cfg, feasibility)
% GPR_EXTRACT_RESULTS  Turn logged test points into a usable results struct.
%
%   res = GPR_EXTRACT_RESULTS(tp, cfg, feasibility) accepts the 14 test
%   points either as plain numeric arrays (run_pipeline_reference) or as
%   timeseries/structure-with-time logs from Simulink (gpr_tp_from_simout)
%   and returns
%
%       .cfg .feasibility .ax        configuration, checks, derived axes
%       .env .tx .tx_chain .channel  named stages of the RF front end
%       .coupled .rx .adc .avg
%       .cal .rp .bs .mig .det       named stages of the digital chain
%       .report                      6x1 vector from B14_Report
%       .depth .x                    depth [m] and cross-range [m] axes
%       .detections                  struct array, one entry per cluster
%       .tp                          the raw test points
%
%   .detections(k) has fields depth, x, n_pixels, amp, depth_min, depth_max.

if nargin < 3
    feasibility = [];
end
res = struct();
res.cfg = cfg;
res.feasibility = feasibility;
res.ax = gpr_axes(cfg);
res.tp = tp;

map = {'env',      'tp01_b01_environment'; ...
       'tx',       'tp02_b02_waveform'; ...
       'tx_chain', 'tp03_b03_tx_chain'; ...
       'channel',  'tp04_b04_channel'; ...
       'coupled',  'tp05_b05_coupling'; ...
       'rx',       'tp06_b06_rx'; ...
       'adc',      'tp07_b07_adc'; ...
       'avg',      'tp08_b08_averaging'; ...
       'cal',      'tp09_b09_calibration'; ...
       'rp',       'tp10_b10_rangeproc'; ...
       'bs',       'tp11_b11_background'; ...
       'mig',      'tp12_b12_migration'; ...
       'det',      'tp13_b13_detection'; ...
       'report',   'tp14_b14_report'};

for k = 1:size(map, 1)
    if isfield(tp, map{k,2})
        res.(map{k,1}) = to_numeric(tp.(map{k,2}));
    else
        res.(map{k,1}) = [];
    end
end
% Second output ports (noise levels) when they were logged too.
res.rx_noise    = pick(tp, 'tp06_b06_rx_o2');
res.adc_noise   = pick(tp, 'tp07_b07_adc_o2');

res.depth = res.ax.depth(:);
res.x     = res.ax.x(:).';

if ~isempty(res.det) && ~isempty(res.mig)
    res.detections = find_detections(res.det, res.mig, res.ax, cfg);
else
    res.detections = struct('depth', {}, 'x', {}, 'n_pixels', {}, ...
        'amp', {}, 'depth_min', {}, 'depth_max', {}, 'x_peak', {}, ...
        'depth_peak', {}, 'x_centroid', {}, 'depth_centroid', {});
end
res.n_detections = numel(res.detections);
res.n_pixels = 0;
if ~isempty(res.det)
    res.n_pixels = sum(res.det(:) > 0);
end
end

% ------------------------------------------------------------------ helpers
function v = pick(tp, name)
if isfield(tp, name)
    v = to_numeric(tp.(name));
else
    v = [];
end
end

function v = to_numeric(x)
if isempty(x)
    v = [];
    return;
end
if isa(x, 'timeseries')
    d = x.Data;
elseif isstruct(x) && isfield(x, 'Data')
    d = x.Data;
elseif isstruct(x) && isfield(x, 'signals') && isfield(x.signals, 'values')
    d = x.signals.values;      % "Structure With Time"
else
    d = x;
end
if ndims(d) > 2
    d = d(:, :, end);          % one-shot model: keep the last sample
end
v = squeeze(d);
if isempty(v)
    v = reshape(d, [], 1);
end
end

function det_list = find_detections(det, mig, ax, cfg)
% Label the 8-connected clusters of the CFAR detection matrix and describe
% each one by its peak position, size and peak migrated amplitude.  The
% reported (depth, x) is the location of the maximum migrated amplitude
% inside the cluster, which is what B14_Report also uses: a detection blob
% carries the migration skirt (and, in a crowded scene, part of a
% neighbour), so the pixel centroid can be biased by more than a metre.
det = double(det > 0);
[nr, nc] = size(det);
[rr, cc] = find(det);
n = numel(rr);
empty = struct('depth', {}, 'x', {}, 'n_pixels', {}, 'amp', {}, ...
    'depth_min', {}, 'depth_max', {}, 'x_peak', {}, 'depth_peak', {}, ...
    'x_centroid', {}, 'depth_centroid', {});
if n == 0
    det_list = empty;
    return;
end

% Sparse lookup from (row, col) to the index in rr/cc.
idx = sparse(rr, cc, (1:n).', nr, nc);

labels = zeros(n, 1);
nlab = 0;
for s = 1:n
    if labels(s) ~= 0
        continue;
    end
    nlab = nlab + 1;
    labels(s) = nlab;
    stack = s;
    while ~isempty(stack)
        cur = stack(end);
        stack(end) = [];
        r0 = rr(cur);
        c0 = cc(cur);
        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0
                    continue;
                end
                r1 = r0 + dr;
                c1 = c0 + dc;
                if r1 < 1 || r1 > nr || c1 < 1 || c1 > nc
                    continue;
                end
                j = idx(r1, c1);
                if j > 0 && labels(j) == 0
                    labels(j) = nlab;
                    stack(end+1) = j; %#ok<AGROW>
                end
            end
        end
    end
end

dx_pos = cfg.scan_length/(cfg.n_positions - 1);
det_list = empty;
for k = 1:nlab
    sel = find(labels == k);
    d = (rr(sel) - 1)*ax.dr;
    xx = (cc(sel) - 1)*dx_pos;
    lin = sub2ind([nr, nc], rr(sel), cc(sel));
    amps = mig(lin);
    [amp_peak, ia] = max(amps);
    det_list(k).depth          = d(ia);              %#ok<AGROW>
    det_list(k).x              = xx(ia);
    det_list(k).n_pixels       = numel(sel);
    det_list(k).amp            = amp_peak;
    det_list(k).depth_min      = min(d);
    det_list(k).depth_max      = max(d);
    det_list(k).x_peak         = xx(ia);
    det_list(k).depth_peak     = d(ia);
    det_list(k).x_centroid     = mean(xx);
    det_list(k).depth_centroid = mean(d);
end

% Biggest clusters first.
sizes = arrayfun(@(s) s.n_pixels, det_list);
[~, order] = sort(sizes, 'descend');
det_list = det_list(order);
end
