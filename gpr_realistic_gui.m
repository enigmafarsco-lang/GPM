function hfig = gpr_realistic_gui(res, varargin)
% GPR_REALISTIC_GUI  Look at the output of gpr_realistic_main.
%
%   gpr_realistic_gui(res)
%   h = gpr_realistic_gui(res, 'profile_x', 6.0, 'show', 'detection')
%
%   res   struct returned by gpr_realistic_main / gpr_extract_results.
%
%   Options
%       'profile_x'  cross-range position [m] of the depth profile
%                    (default: the position of the strongest detection)
%       'show'       which image the big panel draws:
%                    'bscan'     migrated B-scan in dB   (default)
%                    'detection' CFAR detection mask
%                    'raw'       range-processed, background-removed data
%       'figure'     reuse an existing figure handle instead of a new one
%
%   Panels: feasibility/summary text, the image with truth (green circles)
%   and detections (red stars) overlaid, the depth profile at one position
%   and the B14 report vector.
%
%   The figure is built from classic Handle Graphics (figure/subplot/
%   imagesc), not uifigure/uigridlayout, so it also runs in Octave and in
%   MATLAB releases older than R2016a.  No local variable is ever called
%   "grid": that shadows the grid() function and silently breaks every
%   later "grid on" in the same scope.

p = inputParser();
addParameter(p, 'profile_x', []);
addParameter(p, 'show', 'bscan');
addParameter(p, 'figure', []);
parse(p, varargin{:});
opt = p.Results;

if isempty(res) || ~isfield(res, 'mig') || isempty(res.mig)
    error('GPR:gui:noResults', 'res does not contain any migrated data.');
end

% Re-derive the validated configuration: results saved by an older run may
% carry a cfg that predates some derived fields (dx_trace, cfar_alpha, ...).
cfg   = gpr_check_config(res.cfg, res.cfg.mode);
ax    = gpr_axes(cfg);
res.depth = ax.depth(:);
res.x     = ax.x(:).';
depth = res.depth(:);
xpos  = res.x(:).';
nd    = cfg.n_ifft;
nx    = cfg.n_positions;
dxp   = cfg.dx_trace;

if isempty(opt.profile_x)
    if ~isempty(res.detections)
        [~, ia] = max([res.detections.amp]);
        opt.profile_x = res.detections(ia).x;
    else
        opt.profile_x = mean(xpos);
    end
end

if isempty(opt.figure)
    hfig = figure('Name', sprintf('GPR digital twin - %s', cfg.mode), ...
        'NumberTitle', 'off', 'Color', 'w', 'Position', [60 60 1300 780]);
else
    hfig = opt.figure;
    clf(hfig);
end

% Explicit axes positions instead of subplot() with cell spans: the gnuplot
% backend (Octave, head-less servers) drops spanned and invisible axes, while
% MATLAB renders the same positions exactly.
axh(1) = axes('Parent', hfig, 'Position', [0.015 0.03 0.30 0.93]); %#ok<AGROW>
set(axh(1), 'XColor', 'w', 'YColor', 'w', 'XTick', [], 'YTick', []);
title(axh(1), sprintf('%s  (%s engine)', cfg.mode, ...
    gpr_cfg_value(res, 'engine', 'reference')), 'Interpreter', 'none');
% One text object per line: a multi-line/cell text object is drawn on a
% single row by the gnuplot backend, which would stack the summary off-canvas.
lines = summary_lines(res, cfg, ax, opt);
y0 = 0.985; dy = 0.96/max(1, numel(lines));
for k = 1:numel(lines)
    text(axh(1), 0.01, y0 - (k-1)*dy, lines{k}, 'Units', 'normalized', ...
        'FontName', 'Courier', 'FontSize', 8, 'Interpreter', 'none', ...
        'HorizontalAlignment', 'left');
end

% ------------------------------------------------------- 2: the main image
axh(2) = axes('Parent', hfig, 'Position', [0.365 0.40 0.60 0.55]); %#ok<AGROW>
switch lower(opt.show)
    case 'detection'
        img = reshape(double(res.det > 0), [nd, nx]);
        imagesc(axh(2), xpos, depth, img);
        colormap(axh(2), [0 0 0.45; 1 0.85 0.1]);
        ttl = sprintf('CFAR detection mask - %d pixel(s), %d cluster(s)', ...
            res.n_pixels, res.n_detections);
    case 'raw'
        [img, ttl] = scale_db(reshape(res.bs, [nd, nx]), ...
            'range-processed, background removed');
        imagesc(axh(2), xpos, depth, img);
        colormap(axh(2), gray(256));
    otherwise
        [img, ttl] = scale_db(reshape(res.mig, [nd, nx]), 'migrated B-scan');
        imagesc(axh(2), xpos, depth, img);
        colormap(axh(2), gray(256));
end
axis(axh(2), 'xy');
set(axh(2), 'YDir', 'normal');
ylim(axh(2), [0 min(max(depth), 1.15*max(cfg.target_depths))]);
xlabel(axh(2), 'cross range x [m]');
ylabel(axh(2), 'depth [m]');
title(axh(2), ttl, 'Interpreter', 'none');
colorbar(axh(2));
hold(axh(2), 'on');
plot(axh(2), cfg.target_x, cfg.target_depths, 'o', 'MarkerSize', 9, ...
    'LineWidth', 1.6, 'MarkerEdgeColor', [0 0.85 0.2], 'MarkerFaceColor', 'none');
if ~isempty(res.detections)
    dxx = [res.detections.x];
    ddd = [res.detections.depth];
    plot(axh(2), dxx, ddd, '*', 'MarkerSize', 12, ...
        'MarkerEdgeColor', [1 0.15 0.15], 'MarkerFaceColor', [1 0.15 0.15]);
    for k = 1:numel(dxx)
        text(axh(2), dxx(k) + 0.02*cfg.scan_length, ddd(k), sprintf(' %d', k), ...
            'Color', [1 0.2 0.2], 'FontSize', 8, 'FontWeight', 'bold');
    end
    legend(axh(2), {'truth', 'detection'}, 'Location', 'northeast');
else
    legend(axh(2), {'truth'}, 'Location', 'northeast');
end
hold(axh(2), 'off');

% ------------------------------------------------------ 3: the depth profile
[~, ix] = min(abs(xpos - opt.profile_x));
axh(3) = axes('Parent', hfig, 'Position', [0.375 0.08 0.27 0.26]); %#ok<AGROW>
plot(axh(3), depth, img(:, ix), 'b-', 'LineWidth', 1.2);
grid(axh(3), 'on');
xlim(axh(3), [0 min(max(depth), 1.15*max(cfg.target_depths))]);
xlabel(axh(3), 'depth [m]');
ylabel(axh(3), 'amplitude [dB re peak]');
title(axh(3), sprintf('profile at x = %.2f m', xpos(ix)), 'Interpreter', 'none');
hold(axh(3), 'on');
yl = ylim(axh(3));
for t = 1:cfg.n_targets
    if abs(cfg.target_x(t) - xpos(ix)) < 1.5*dxp
        plot(axh(3), [cfg.target_depths(t) cfg.target_depths(t)], yl, ...
            'g--', 'LineWidth', 1.2);
    end
end
hold(axh(3), 'off');

% ------------------------------------------------------------ 4: the report
axh(4) = axes('Parent', hfig, 'Position', [0.71 0.08 0.27 0.26]); %#ok<AGROW>
rv = res.report(:);
rl = {'pixels', 'clusters', 'mean d', 'mean x', 'min d', 'max d'};
nv = min(6, numel(rv));
bar(axh(4), rv(1:nv), 'FaceColor', [0.2 0.45 0.75]);
set(axh(4), 'XTick', 1:nv, 'XTickLabel', rl(1:nv), ...
    'TickLabelInterpreter', 'none');
ylabel(axh(4), 'value');
title(axh(4), 'B14 report vector', 'Interpreter', 'none');
grid(axh(4), 'on');
hold(axh(4), 'on');
for k = 1:nv
    text(axh(4), k, rv(k), sprintf('%.3g', rv(k)), 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
hold(axh(4), 'off');

drawnow();
end

% ------------------------------------------------------------------ helpers
function [img, ttl] = scale_db(img, what)
a = abs(img);
mx = max(a(:));
if ~(mx > 0), mx = 1; end
img = 20*log10(max(a, mx*1e-6)/mx);
img(img < -60) = -60;
ttl = sprintf('%s [dB re peak, clipped at -60 dB]', what);
end

function txt = summary_lines(res, cfg, ax, opt)
L = {};
L{end+1} = cfg.description; %#ok<AGROW>
L{end+1} = sprintf('band       %6.0f - %6.0f MHz, %d tones', ...
    cfg.f_start/1e6, cfg.f_stop/1e6, cfg.n_tones);
L{end+1} = sprintf('survey     %d positions over %.1f m (dx = %.3f m)', ...
    cfg.n_positions, cfg.scan_length, cfg.dx_trace);
L{end+1} = sprintf('standoff   %.3f m   eps_r %.2f   v %.3e m/s   %.2f dB/m', ...
    cfg.uav_altitude, cfg.soil_eps_r_est, cfg.soil_v, cfg.soil_alpha_dB);
L{end+1} = sprintf('range res  %.4f m    depth res (kaiser b=%.1f) %.4f m', ...
    ax.range_res, cfg.kaiser_beta, cfg.depth_resolution);
L{end+1} = sprintf('depth      %.2f m unambiguous, %.2f m IFFT window', ...
    ax.unambig_depth, ax.max_ifft_depth);
L{end+1} = sprintf('CFAR       %s, Pfa %.1e, %dx%d cells, %d ref', ...
    cfg.cfar_estimator, cfg.cfar_pfa, 2*cfg.cfar_guard+1, ...
    2*cfg.cfar_train+1, cfg.cfar_n_ref);
L{end+1} = sprintf('threshold  %.2f x local level = %.1f dB', ...
    cfg.cfar_alpha, cfg.cfar_threshold_dB);
L{end+1} = sprintf('migration  aperture %.2f, %d traces/Fresnel zone', ...
    cfg.migration_aperture, cfg.fresnel_traces);
L{end+1} = '';
L{end+1} = sprintf('DETECTIONS %d cluster(s), %d pixel(s)', ...
    res.n_detections, res.n_pixels);
for k = 1:cfg.n_targets
    if isfield(res, 'match') && ~isnan(res.match(k, 1))
        L{end+1} = sprintf('  T%d x %6.2f (%+6.3f)  d %6.2f (%+6.3f) m', k, ...
            cfg.target_x(k) + res.match(k,1), res.match(k,1), ...
            cfg.target_depths(k) + res.match(k,2), res.match(k,2));
    else
        L{end+1} = sprintf('  T%d MISSED  (truth x %.2f, d %.2f m)', k, ...
            cfg.target_x(k), cfg.target_depths(k));
    end
end
if ~isempty(res.feasibility)
    L{end+1} = '';
    if res.feasibility.ok
        L{end+1} = 'PHYSICS    FEASIBLE';
    else
        L{end+1} = 'PHYSICS    NOT FEASIBLE';
    end
    for k = 1:numel(res.feasibility.errors)
        L{end+1} = sprintf('  ERROR  %s', res.feasibility.errors{k});
    end
    for k = 1:numel(res.feasibility.warnings)
        L{end+1} = sprintf('  warn   %s', res.feasibility.warnings{k});
    end
end
L{end+1} = '';
L{end+1} = sprintf('showing: %s, profile at x = %.2f m', opt.show, opt.profile_x);
txt = L(:);   % cellstr: one text object per line, renders in MATLAB and Octave
end
