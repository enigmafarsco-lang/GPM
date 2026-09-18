function feas = check_physics_feasibility(cfg)
% CHECK_PHYSICS_FEASIBILITY  Link-budget and sampling sanity checks.

feas = struct('viable', true, 'reason', '', 'suggestions', {{}});
if cfg.n_tones < 2 || cfg.n_ifft < cfg.n_tones
    error('Invalid sampling dimensions: n_tones >= 2 and n_ifft >= n_tones are required.');
end

v_soil = cfg.c / sqrt(cfg.soil_eps_r_est);
range_res = v_soil / (2*cfg.bw);
unambig = v_soil / (2*cfg.delta_f);
dt = 1/(cfg.n_ifft*cfg.delta_f);
max_ifft_depth = cfg.n_ifft*dt*v_soil/2;
max_tgt_depth = max(cfg.target_depths);

fprintf('  Band: %.1f-%.1f MHz, tones: %d, Δf: %.4f MHz\n', ...
    cfg.f_start/1e6, cfg.f_stop/1e6, cfg.n_tones, cfg.delta_f/1e6);
fprintf('  Soil velocity: %.3e m/s, range resolution: %.3f m\n', v_soil, range_res);
fprintf('  Unambiguous depth: %.2f m, IFFT window: %.2f m\n', unambig, max_ifft_depth);

feas.v_soil = v_soil;
feas.range_res = range_res;
feas.unambig = unambig;
feas.max_ifft_depth = max_ifft_depth;

if unambig < 1.2*max_tgt_depth
    feas = fail(feas, sprintf('Unambiguous depth %.2f m is too small for %.2f m target depth.', unambig, max_tgt_depth), ...
        sprintf('Increase tone count or reduce the deepest target to below %.2f m.', unambig/1.2));
end
if max_ifft_depth < 1.1*max_tgt_depth
    feas = fail(feas, sprintf('IFFT window %.2f m truncates the %.2f m target.', max_ifft_depth, max_tgt_depth), ...
        'Increase n_ifft or reduce the requested depth window.');
end

% For ground-coupled mode, antenna separation is the relevant air/near-field
% scale. UAV altitude is not used as a substitute for it.
if isfield(cfg,'mode') && startsWith(cfg.mode,'B_')
    R_air = max(cfg.tx_rx_separation/2, 0.10);
else
    R_air = max(cfg.uav_altitude, 0.10);
end
lambda = cfg.c/cfg.f_center;
two_way_air_dB = 2*20*log10(4*pi*R_air/lambda);
B_eff = cfg.bw;
noise_dBm = -174 + 10*log10(B_eff) + cfg.system_nf_dB;
integ_dB = 10*log10(cfg.n_averages*cfg.n_tones);

fprintf('  Link-budget path scale: %.3f m, two-way air/near-field term: %.1f dB\n', R_air, two_way_air_dB);
for t = 1:cfg.n_targets
    depth = cfg.target_depths(t);
    omega = 2*pi*cfg.f_center;
    eps_i = cfg.soil_conductivity/(omega*cfg.eps0);
    lt = eps_i/cfg.soil_eps_r_est;
    alpha = omega*sqrt(cfg.mu0*cfg.eps0*cfg.soil_eps_r_est/2) * sqrt(sqrt(1+lt^2)-1);
    soil_loss = 2*depth*alpha*20/log(10);
    rcs_dBsm = 10*log10(max(cfg.target_rcs(t),eps));
    p_rx = cfg.eirp_dBm - soil_loss - two_way_air_dB - 6 + rcs_dBsm + ...
        cfg.rx_antenna_gain_dBi + integ_dB;
    snr = p_rx - noise_dBm;
    fprintf('  Target %d: depth %.2f m, estimated SNR %.1f dB\n', t, depth, snr);
    if snr < 10
        feas = fail(feas, sprintf('Target %d is below the 10 dB estimated detection margin.', t), ...
            'Reduce soil loss, increase coherent integration, or use a larger target RCS.');
    end
end
if feas.viable
    fprintf('  Feasibility: PASS\n');
else
    fprintf('  Feasibility: WARNING\n');
end
end

function feas = fail(feas, reason, suggestion)
if feas.viable, feas.reason = reason; end
feas.viable = false;
feas.suggestions{end+1} = suggestion;
end
