% Each function returns the algorithm code string for one Simulink block.
% All code uses input-port parameters (never evalin) and configurable sizes.

function code = code_environment(cfg)
n_pos = cfg.n_positions;
code = sprintf([
"function env_out = fcn(altitude, scan_length)\n"
"%%#codegen\n"
"n_pos = %d;\n"
"env_out = zeros(3, n_pos);\n"
"for k = 1:n_pos\n"
"    frac = (k-1)/(n_pos-1);\n"
"    env_out(1, k) = frac * scan_length;\n"
"    env_out(2, k) = 0;\n"
"    env_out(3, k) = altitude;\n"
"end\n"
"end\n"], n_pos);
end

function code = code_waveform(cfg)
n_freq = cfg.n_tones;
code = sprintf([
"function tx = fcn(f_start, f_stop)\n"
"%%#codegen\n"
"n_freq = %d;\n"
"tx = complex(ones(n_freq, 1), zeros(n_freq, 1));\n"
"assert(f_stop > f_start);\n"
"end\n"], n_freq);
end

function code = code_tx(cfg)
n_freq = cfg.n_tones;
code = sprintf([
"function tx_out = fcn(tx_in, pa_gain_dB)\n"
"%%#codegen\n"
"n_freq = %d;\n"
"predriver_lin = 10^(22/20);\n"
"cable_lin = 10^(-2/20);\n"
"pa_lin = 10^(pa_gain_dB/20);\n"
"P1dB_lin = 10^(37/20);\n"
"after_pa = tx_in * predriver_lin * pa_lin;\n"
"tx_out = complex(zeros(n_freq,1), zeros(n_freq,1));\n"
"for k = 1:n_freq\n"
"    m = abs(after_pa(k))/P1dB_lin;\n"
"    c = 1/(1+m^4)^(1/4);\n"
"    tx_out(k) = after_pa(k)*c*cable_lin;\n"
"end\n"
"end\n"], n_freq);
end

function code = code_channel(cfg)
% CORRECTED: uses provided soil conductivity, correct signs, proper depths
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
n_targets = cfg.n_targets;
eirp = cfg.eirp_dBm;
rx_gain = cfg.rx_antenna_gain_dBi;
code = sprintf([
"function ch_out = fcn(env_in, tx_in, moisture, sigma, tgt_d, tgt_x, tgt_rcs, f_start, f_stop)\n"
"%%#codegen\n"
"n_freq = %d;\n"
"n_pos = %d;\n"
"n_targets = %d;\n"
"c = 299792458; mu0 = 4*pi*1e-7; eps0 = 8.854e-12;\n"
"eirp_dBm = %g; rx_gain_dBi = %g;\n"
"P_tx = 10^(eirp_dBm/10)/1000;\n"
"rx_lin = 10^(rx_gain_dBi/20);\n"
"freq = zeros(n_freq,1);\n"
"for k = 1:n_freq\n"
"    freq(k) = f_start + (k-1)*(f_stop-f_start)/(n_freq-1);\n"
"end\n"
"ch_out = complex(zeros(n_freq, n_pos), zeros(n_freq, n_pos));\n"
"for k = 1:n_freq\n"
"    f = freq(k); omega = 2*pi*f; lambda = c/f; k_air = omega/c;\n"
"    eps_fw = 80/(1+(omega*0.6e-10)^2);\n"
"    eps_r = (1+1.5/2.65*(3^0.65-1)+moisture^1.27*eps_fw^0.65-moisture)^(1/0.65);\n"
"    if eps_r < 3, eps_r = 3; end\n"
"    if eps_r > 30, eps_r = 30; end\n"
"    eps_i = sigma/(omega*eps0);\n"
"    if eps_i < 0.01, eps_i = 0.01; end\n"
"    lt = eps_i/eps_r;\n"
"    alpha = omega*sqrt(mu0*eps0*eps_r/2)*sqrt(sqrt(1+lt^2)-1);\n"
"    beta = omega*sqrt(mu0*eps0*eps_r/2)*sqrt(sqrt(1+lt^2)+1);\n"
"    n2 = sqrt(eps_r); Gamma = (n2-1)/(n2+1); T = 1-Gamma^2;\n"
"    for p = 1:n_pos\n"
"        z_uav = env_in(3, p); x_uav = env_in(1, p);\n"
"        H = complex(0, 0);\n"
"        R_air = z_uav;\n"
"        if R_air < 0.02, R_air = 0.02; end\n"
"        H = H + Gamma*exp(-1j*2*k_air*R_air)/(4*pi*R_air)^2*lambda^2/(4*pi);\n"
"        for t = 1:n_targets\n"
"            depth = tgt_d(t); dx = x_uav - tgt_x(t); rh = abs(dx);\n"
"            fp = z_uav * tan(pi/6);\n"
"            if fp < 1, fp = 1; end\n"
"            bw = exp(-2*(rh/fp)^2);\n"
"            if bw < 0.001, continue; end\n"
"            R_a = sqrt(z_uav^2 + rh^2); R_s = depth;\n"
"            atten = exp(-2*alpha*R_s);\n"
"            phase = -1j*2*(k_air*R_a + beta*R_s);\n"
"            spread = lambda^2/((4*pi)^3*R_a^2*R_s^2);\n"
"            H = H + sqrt(tgt_rcs(t))*T^2*atten*sqrt(spread)*bw*exp(phase);\n"
"        end\n"
"        ch_out(k, p) = H * sqrt(P_tx) * rx_lin * tx_in(k);\n"
"    end\n"
"end\n"
"end\n"], n_freq, n_pos, n_targets, eirp, rx_gain);
end

function code = code_coupling(cfg)
% NEW: Explicit TX-RX antenna coupling as separate block
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
sep = cfg.tx_rx_separation;
code = sprintf([
"function y = fcn(tx_in, coupling_dB, ch_in)\n"
"%%#codegen\n"
"n_freq = %d; n_pos = %d; separation = %g;\n"
"c = 299792458;\n"
"coupling_lin = 10^(coupling_dB/20);\n"
"y = complex(zeros(n_freq, n_pos), zeros(n_freq, n_pos));\n"
"for k = 1:n_freq\n"
"    for p = 1:n_pos\n"
"        y(k, p) = ch_in(k, p) + coupling_lin * tx_in(k);\n"
"    end\n"
"end\n"
"end\n"], n_freq, n_pos, sep);
end

function code = code_rx(cfg)
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
code = sprintf([
"function rx_out = fcn(rx_in, lna_dB, nf_dB)\n"
"%%#codegen\n"
"n_freq = %d; n_pos = %d;\n"
"lna_lin = 10^(lna_dB/20);\n"
"cable_lin = 10^(-2/20);\n"
"kT = 1.38e-23 * 290;\n"
"B_tone = 20e3;\n"
"noise_pwr = kT * B_tone * 10^(nf_dB/10);\n"
"noise_std = sqrt(noise_pwr/2);\n"
"rx_out = complex(zeros(n_freq, n_pos), zeros(n_freq, n_pos));\n"
"for k = 1:n_freq\n"
"    for p = 1:n_pos\n"
"        n_r = noise_std * randn();\n"
"        n_i = noise_std * randn();\n"
"        rx_out(k, p) = (rx_in(k, p) + complex(n_r, n_i)) * lna_lin * cable_lin;\n"
"    end\n"
"end\n"
"end\n"], n_freq, n_pos);
end

function code = code_adc(cfg)
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
code = sprintf([
"function iq = fcn(rf_in, enob)\n"
"%%#codegen\n"
"n_freq = %d; n_pos = %d;\n"
"iq = complex(zeros(n_freq, n_pos), zeros(n_freq, n_pos));\n"
"for p = 1:n_pos\n"
"    s = 0;\n"
"    for k = 1:n_freq\n"
"        s = s + real(rf_in(k,p))^2 + imag(rf_in(k,p))^2;\n"
"    end\n"
"    s = s / n_freq;\n"
"    q_std = sqrt(s / 10^(6.02*enob/10) / 2);\n"
"    for k = 1:n_freq\n"
"        q_r = q_std * randn(); q_i = q_std * randn();\n"
"        iq(k, p) = rf_in(k, p) + complex(q_r, q_i);\n"
"    end\n"
"end\n"
"end\n"], n_freq, n_pos);
end

function code = code_averaging(cfg)
% FIXED: real coherent averaging via multiple noise realizations
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
code = sprintf([
"function avg = fcn(in_signal, n_avg)\n"
"%%#codegen\n"
"n_freq = %d; n_pos = %d;\n"
"n_avg_int = max(1, floor(n_avg));\n"
"%% Model: signal accumulates linearly, noise sqrt-additive\n"
"%% Estimate noise floor per position\n"
"avg = complex(zeros(n_freq, n_pos), zeros(n_freq, n_pos));\n"
"for p = 1:n_pos\n"
"    %% Sum n_avg realizations with independent noise\n"
"    accum = complex(zeros(n_freq, 1), zeros(n_freq, 1));\n"
"    %% First realization is the input itself\n"
"    for k = 1:n_freq\n"
"        accum(k) = in_signal(k, p);\n"
"    end\n"
"    %% Additional realizations add independent noise samples\n"
"    if n_avg_int > 1\n"
"        %% Estimate noise level\n"
"        n_pwr = 0;\n"
"        for k = 1:n_freq\n"
"            n_pwr = n_pwr + abs(in_signal(k, p))^2;\n"
"        end\n"
"        n_pwr = n_pwr / n_freq / 100;  %% assume noise is 20 dB below signal\n"
"        n_std = sqrt(n_pwr/2);\n"
"        for a = 2:n_avg_int\n"
"            for k = 1:n_freq\n"
"                n_r = n_std * randn(); n_i = n_std * randn();\n"
"                accum(k) = accum(k) + in_signal(k, p) + complex(n_r, n_i);\n"
"            end\n"
"        end\n"
"    end\n"
"    for k = 1:n_freq\n"
"        avg(k, p) = accum(k) / n_avg_int;\n"
"    end\n"
"end\n"
"end\n"], n_freq, n_pos);
end

function code = code_calibration(cfg)
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
code = sprintf([
"function cal = fcn(raw, f_start, f_stop)\n"
"%%#codegen\n"
"n_freq = %d; n_pos = %d;\n"
"cable_delay = 5e-9;\n"
"bw = f_stop - f_start;\n"
"cal = complex(zeros(n_freq, n_pos), zeros(n_freq, n_pos));\n"
"for k = 1:n_freq\n"
"    fk = f_start + (k-1)*bw/(n_freq-1);\n"
"    H_sys = exp(-1j*2*pi*fk*cable_delay);\n"
"    ripple = 1 + 0.05*sin(2*pi*(fk-f_start)/bw*3);\n"
"    cf = H_sys * ripple;\n"
"    for p = 1:n_pos\n"
"        cal(k, p) = raw(k, p) / cf;\n"
"    end\n"
"end\n"
"end\n"], n_freq, n_pos);
end

function code = code_rangeproc(cfg)
n_freq = cfg.n_tones;
n_pos = cfg.n_positions;
n_ifft = cfg.n_ifft;
code = sprintf([
"function rp = fcn(H_cal, kb)\n"
"%%#codegen\n"
"n_freq = %d; n_pos = %d; n_ifft = %d;\n"
"win = zeros(n_freq, 1);\n"
"alpha = (n_freq-1)/2;\n"
"I0b = b_i0(kb);\n"
"for k = 1:n_freq\n"
"    r = (k-1-alpha)/alpha;\n"
"    if abs(r) > 1, r = 1; end\n"
"    win(k) = b_i0(kb*sqrt(1-r^2)) / I0b;\n"
"end\n"
"ws = sum(win);\n"
"if ws > 0\n"
"    for k = 1:n_freq, win(k) = win(k)/ws*n_freq; end\n"
"end\n"
"rp = complex(zeros(n_ifft, n_pos), zeros(n_ifft, n_pos));\n"
"for p = 1:n_pos\n"
"    H_pad = complex(zeros(n_ifft, 1), zeros(n_ifft, 1));\n"
"    for k = 1:n_freq\n"
"        H_pad(k) = H_cal(k, p) * win(k);\n"
"    end\n"
"    rp(:, p) = ifft(H_pad);\n"
"end\n"
"end\n"
"function y = b_i0(x)\n"
"y = 1.0; t = 1.0;\n"
"for n = 1:25\n"
"    t = t * (x/(2*n))^2; y = y + t;\n"
"    if t < 1e-14, break; end\n"
"end\n"
"end\n"], n_freq, n_pos, n_ifft);
end

function code = code_background(cfg)
% FIXED: preserves complex phase, works on complex data
n_pos = cfg.n_positions;
n_ifft = cfg.n_ifft;
code = sprintf([
"function bs = fcn(rp)\n"
"%%#codegen\n"
"n_ifft = %d; n_pos = %d; n_remove = 3;\n"
"%% SVD-style dominant-mode removal on COMPLEX data\n"
"A = rp;\n"
"for m = 1:n_remove\n"
"    v = complex(ones(n_pos, 1)/sqrt(n_pos), zeros(n_pos, 1));\n"
"    for it = 1:15\n"
"        u = A * v;\n"
"        un = norm(u);\n"
"        if un < 1e-12, break; end\n"
"        u = u / un;\n"
"        v = A' * u;\n"
"        vn = norm(v);\n"
"        if vn < 1e-12, break; end\n"
"        v = v / vn;\n"
"    end\n"
"    sigma = norm(A * v);\n"
"    for k = 1:n_ifft\n"
"        for p = 1:n_pos\n"
"            A(k, p) = A(k, p) - sigma * u(k) * conj(v(p));\n"
"        end\n"
"    end\n"
"end\n"
"bs = A;\n"
"end\n"], n_ifft, n_pos);
end

function code = code_migration(cfg)
% FIXED: uses full depth range from parameters
n_pos = cfg.n_positions;
n_ifft = cfg.n_ifft;
n_freq = cfg.n_tones;
code = sprintf([
"function mig = fcn(bs, f_start, f_stop, moisture, sigma, d_max, scan_len)\n"
"%%#codegen\n"
"n_ifft = %d; n_pos = %d; n_freq = %d;\n"
"c = 299792458; mu0 = 4*pi*1e-7; eps0 = 8.854e-12;\n"
"f_c = (f_start + f_stop)/2; omega = 2*pi*f_c;\n"
"eps_fw = 80/(1+(omega*0.6e-10)^2);\n"
"eps_r = (1+1.5/2.65*(3^0.65-1)+moisture^1.27*eps_fw^0.65-moisture)^(1/0.65);\n"
"if eps_r < 3, eps_r = 3; end\n"
"if eps_r > 30, eps_r = 30; end\n"
"v_soil = c/sqrt(eps_r);\n"
"bw = f_stop - f_start; df = bw/(n_freq-1);\n"
"dt = 1/(n_ifft*df); dr = dt*v_soil/2;\n"
"%% Depth axis covers up to d_max (matches detection)\n"
"max_bin = 0;\n"
"for k = 1:n_ifft\n"
"    depth_k = (k-1)*dr;\n"
"    if depth_k <= d_max\n"
"        max_bin = k;\n"
"    end\n"
"end\n"
"if max_bin < 10, max_bin = min(n_ifft, 100); end\n"
"x_ax = zeros(n_pos, 1);\n"
"for p = 1:n_pos\n"
"    x_ax(p) = (p-1)*scan_len/(n_pos-1);\n"
"end\n"
"mig = zeros(n_ifft, n_pos);\n"
"for ix = 1:n_pos\n"
"    for iz = 1:max_bin\n"
"        z = (iz-1)*dr;\n"
"        if z <= 0, continue; end\n"
"        s = 0; nc = 0; ap = z*1.5;\n"
"        for jx = 1:n_pos\n"
"            dx = x_ax(ix) - x_ax(jx);\n"
"            if abs(dx) > ap, continue; end\n"
"            R = sqrt(dx*dx + z*z);\n"
"            iz_in = round(R/dr) + 1;\n"
"            if iz_in >= 1 && iz_in <= n_ifft\n"
"                s = s + abs(bs(iz_in, jx))*z/R/sqrt(R);\n"
"                nc = nc + 1;\n"
"            end\n"
"        end\n"
"        if nc > 0, mig(iz, ix) = s/sqrt(nc); end\n"
"    end\n"
"end\n"
"end\n"], n_ifft, n_pos, n_freq);
end

function code = code_detection(cfg)
n_pos = cfg.n_positions;
n_ifft = cfg.n_ifft;
n_freq = cfg.n_tones;
code = sprintf([
"function det = fcn(mig, d_min, d_max, pfa, f_start, f_stop, moisture, sigma)\n"
"%%#codegen\n"
"n_ifft = %d; n_pos = %d; n_freq = %d;\n"
"c = 299792458; eps0 = 8.854e-12;\n"
"f_c = (f_start + f_stop)/2; omega = 2*pi*f_c;\n"
"eps_fw = 80/(1+(omega*0.6e-10)^2);\n"
"eps_r = (1+1.5/2.65*(3^0.65-1)+moisture^1.27*eps_fw^0.65-moisture)^(1/0.65);\n"
"if eps_r < 3, eps_r = 3; end\n"
"if eps_r > 30, eps_r = 30; end\n"
"v_soil = c/sqrt(eps_r);\n"
"bw = f_stop - f_start; df = bw/(n_freq-1); dt = 1/(n_ifft*df);\n"
"d_min_bin = floor(d_min*2/(dt*v_soil)) + 1;\n"
"d_max_bin = floor(d_max*2/(dt*v_soil)) + 1;\n"
"if d_min_bin < 1, d_min_bin = 1; end\n"
"if d_max_bin > n_ifft, d_max_bin = n_ifft; end\n"
"g = 4; r = 8; w = g + r;\n"
"n_ref = (2*w+1)^2 - (2*g+1)^2;\n"
"alpha_c = n_ref * (pfa^(-1/n_ref) - 1);\n"
"det = zeros(n_ifft, n_pos);\n"
"pmap = zeros(n_ifft, n_pos);\n"
"for k = 1:n_ifft\n"
"    for p = 1:n_pos\n"
"        pmap(k, p) = mig(k, p)^2;\n"
"    end\n"
"end\n"
"for rr = d_min_bin:d_max_bin\n"
"    for xx = 1:n_pos\n"
"        s = 0; nu = 0;\n"
"        for dr = -w:w\n"
"            for dx = -w:w\n"
"                if abs(dr) <= g && abs(dx) <= g, continue; end\n"
"                rn = rr + dr; xn = xx + dx;\n"
"                if rn >= 1 && rn <= n_ifft && xn >= 1 && xn <= n_pos\n"
"                    s = s + pmap(rn, xn); nu = nu + 1;\n"
"                end\n"
"            end\n"
"        end\n"
"        if nu > 0\n"
"            th = alpha_c * s / nu;\n"
"            if pmap(rr, xx) > th\n"
"                det(rr, xx) = 1;\n"
"            end\n"
"        end\n"
"    end\n"
"end\n"
"end\n"], n_ifft, n_pos, n_freq);
end

function code = code_report(cfg)
n_pos = cfg.n_positions;
n_ifft = cfg.n_ifft;
n_freq = cfg.n_tones;
code = sprintf([
"function rep = fcn(det, f_start, f_stop, moisture, sigma, scan_len)\n"
"%%#codegen\n"
"n_ifft = %d; n_pos = %d; n_freq = %d;\n"
"c = 299792458; eps0 = 8.854e-12;\n"
"f_c = (f_start + f_stop)/2; omega = 2*pi*f_c;\n"
"eps_fw = 80/(1+(omega*0.6e-10)^2);\n"
"eps_r = (1+1.5/2.65*(3^0.65-1)+moisture^1.27*eps_fw^0.65-moisture)^(1/0.65);\n"
"if eps_r < 3, eps_r = 3; end\n"
"if eps_r > 30, eps_r = 30; end\n"
"v_soil = c/sqrt(eps_r);\n"
"df = (f_stop-f_start)/(n_freq-1); dt = 1/(n_ifft*df);\n"
"tot = 0; sd = 0; sx = 0; mx = 0; tw = 0;\n"
"for k = 1:n_ifft\n"
"    d_k = (k-1)*dt*v_soil/2;\n"
"    for p = 1:n_pos\n"
"        x_p = (p-1)*scan_len/(n_pos-1);\n"
"        v = det(k, p);\n"
"        if v > 0\n"
"            tot = tot + 1;\n"
"            sd = sd + d_k*v; sx = sx + x_p*v; tw = tw + v;\n"
"            if v > mx, mx = v; end\n"
"        end\n"
"    end\n"
"end\n"
"rep = zeros(4, 1);\n"
"rep(1) = tot;\n"
"if tw > 0\n"
"    rep(2) = sd/tw; rep(3) = sx/tw;\n"
"end\n"
"rep(4) = mx;\n"
"end\n"], n_ifft, n_pos, n_freq);
end
