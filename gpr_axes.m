function ax = gpr_axes(cfg)
% GPR_AXES  Derived frequency / range / cross-range axes for a configuration.
%
%   AX = GPR_AXES(cfg) returns a struct with
%       .freq      tone frequencies, Hz                 (1 x n_tones)
%       .df        tone spacing, Hz
%       .dt        IFFT sample interval, s
%       .dr        depth bin size, m
%       .eps_r     soil relative permittivity at f_center
%       .alpha     soil attenuation at f_center, Np/m
%       .v_soil    phase velocity in soil, m/s
%       .depth     depth of every IFFT bin, m           (1 x n_ifft)
%       .x         cross-range position of every trace, m (1 x n_positions)
%       .range_res unambiguous_depth, max_ifft_depth (m)
%
%   This is the single place where the range/depth scaling of the pipeline is
%   defined, and it matches the formulas baked into the generated blocks.

if nargin < 1 || isempty(cfg)
    error('gpr_axes:badInput', 'A configuration struct is required.');
end

c0 = 299792458;
n_freq = cfg.n_tones;
n_pos  = cfg.n_positions;
n_ifft = cfg.n_ifft;

df = (cfg.f_stop - cfg.f_start)/(n_freq - 1);
dt = 1/(n_ifft*df);

[eps_r, ~, alpha, ~, v_soil] = soil_permittivity(cfg.f_center, cfg.soil_moisture, cfg.soil_conductivity);

dr = dt*v_soil/2;

ax.freq   = cfg.f_start + (0:n_freq-1)*df;
ax.df     = df;
ax.dt     = dt;
ax.dr     = dr;
ax.eps_r  = eps_r;
ax.alpha  = alpha;
ax.v_soil = v_soil;
ax.depth  = (0:n_ifft-1)*dr;
ax.x      = (0:n_pos-1)*cfg.scan_length/(n_pos-1);

ax.range_res      = v_soil/(2*cfg.bw);
ax.unambig_depth  = v_soil/(2*df);
ax.max_ifft_depth = n_ifft*dt*v_soil/2;
ax.c0 = c0;
end
