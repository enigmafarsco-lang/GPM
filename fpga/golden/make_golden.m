function make_golden(outdir)
%MAKE_GOLDEN  Generate every ROM image, register value and test vector the
% RTL package consumes, straight from the validated MATLAB package (configs
% + code_* generators).  Run in MATLAB or Octave from anywhere:
%
%     addpath('<repo>');            % the GPM package root
%     addpath('<repo>/fpga/golden');
%     make_golden                   % writes into <repo>/fpga/golden
%
% The RTL never re-derives a physical constant: if a number changes in
% config_mode_*.m or code_*.m, re-run this script and the hardware image
% follows.  Integer arithmetic mirrors the RTL bit-for-bit where exactness
% is claimed (CFAR, clustering, background), and float reference where the
% testbench uses a tolerance (FFT, migration).

if nargin < 1
    here = fileparts(mfilename('fullpath'));
    outdir = here;
end
if ~exist(outdir, 'dir'), mkdir(outdir); end
repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));  % .../GPM
addpath(repo);

F_CLK   = 245760000;      % fabric/RFDC AXIS clock [Hz]
FW_BITS = 48;             % DDS phase accumulator width
cfgA = gpr_check_config(config_mode_A_uav_shallow(), 'A');
cfgB = gpr_check_config(config_mode_B_ground_deep(), 'B');

%% ------------------------------------------------------------- sine ROM
% quarter-wave, 4096 entries, Q15
n = (0:4095)';
s = round(sin((n + 0.5)/4096*pi/2)*32767);
write_hex(fullfile(outdir, 'sine_rom.hex'), s, 4);

%% ------------------------------------------- per-mode constants + ROMs
modes = {cfgA, 'a'; cfgB, 'b'};
for mi = 1:2
    cfg = modes{mi,1}; tag = modes{mi,2};
    NT = 256;
    f0 = cfg.f_start; f1 = cfg.f_stop;
    fk = f0 + (0:NT-1)'*((f1 - f0)/(NT - 1));

    % ---- sub-band tone plan (RFDC mixer span = fabric Nyquist) --------
    span = F_CLK/2;                      % +-122.88 MHz around the LO
    n_sb = ceil((f1 - f0)/span);
    sb   = min(n_sb, floor((fk - f0)/span) + 1);   % sub-band per tone
    [sbo, ord] = sort(sb, 'ascend');               % stable: keeps f order
    fk_seq  = fk(ord);
    tidx_seq = ord - 1;                            % true tone index, 0-based
    lo_seq  = f0 + (sbo - 1)*span + span/2;        % sub-band centre = LO
    beat    = fk_seq - lo_seq;                     % |beat| <= span/2
    fw = mod(round(beat/F_CLK*2^FW_BITS), 2^FW_BITS);
    write_hex(fullfile(outdir, ['fw_seq_' tag '.hex']), fw, 12);
    write_hex(fullfile(outdir, ['tidx_seq_' tag '.hex']), tidx_seq, 2);
    sb_ends = zeros(1, n_sb); c = 0;
    for q = 1:n_sb
        c = c + sum(sbo == q);
        sb_ends(q) = c - 1;            % 0-based last tone index of sub-band
    end
    write_hex_row(fullfile(outdir, ['sb_ends_' tag '.hex']), sb_ends, 4);

    % ---- TX amplitude ROM (true tone order): B02+B03 from the package --
    wscr  = code_waveform(cfg); txscr = code_tx(cfg);
    wf  = gpr_compile_script(wscr, 'gold_waveform', fullfile(outdir, 'tmp'));
    txf = gpr_compile_script(txscr, 'gold_tx', fullfile(outdir, 'tmp'));
    tx_out = txf(wf(f0, f1), cfg.pa_gain_dB);
    amp = round(real(tx_out)*32767);           % Q15, |.| <= 1 after norm
    write_hex(fullfile(outdir, ['amp_' tag '.hex']), amp, 4);

    % ---- calibration ROM (B09): 1/H_sys, Q14 complex ------------------
    cable_delay = cfg.rx_cable_delay; ripple_depth = 0.05; ripple_cycles = 3;
    bw = f1 - f0; df = bw/(NT - 1);
    Hs = exp(-1j*2*pi*fk*cable_delay) .* ...
         (1 + ripple_depth*sin(2*pi*(fk - f0)/bw*ripple_cycles));
    inv_H = 1./Hs;
    inv_H(abs(inv_H) > 1.99) = 1.99*inv_H(abs(inv_H) > 1.99)./abs(inv_H(abs(inv_H) > 1.99));
    cal = bitor_shift(round(real(inv_H)*2^14), round(imag(inv_H)*2^14));
    write_hex(fullfile(outdir, ['cal_' tag '.hex']), cal, 8);

    % ---- derived physical registers ------------------------------------
    [~, ~, alpha_np, ~, v] = soil_permittivity((f0 + f1)/2, cfg.soil_moisture, cfg.soil_conductivity);
    n_ifft = cfg.n_ifft; n_pos = cfg.n_positions;
    dr = v/(2*n_ifft*df);
    dx = cfg.scan_length/(n_pos - 1);
    c0 = 299792458;
    r_off   = cfg.uav_altitude*(v/c0);
    kbeta   = 2*f0/v;                        % phase turns per metre
    max_bin = min(n_ifft, floor(cfg.depth_max/dr) + 2);
    % aperture plan -> nap per depth bin (exact float, dumped as ROM)
    tan_beam = 0.57735; fp_min = 0.3;
    lambda0 = v/f0; lambda_c = v/((f0+f1)/2);
    sin_max = min(0.95, lambda0/(8*dx));
    tan_alias = sin_max/sqrt(max(1e-6, 1 - sin_max^2));
    tan_spot = lambda_c/(4*dx);
    tan_lim = min(tan_alias, tan_spot);
    nap = zeros(max_bin, 1);
    for iz = 1:max_bin
        z = (iz - 1)*dr;
        if z <= 0, continue; end
        ap = min([z, max((cfg.uav_altitude + z)*tan_beam, fp_min), tan_lim*z]);
        nap(iz) = max(1, floor(ap/dx));
    end
    write_hex(fullfile(outdir, ['nap_' tag '.hex']), nap(1:max_bin), 2);

    % ---- CFAR registers -------------------------------------------------
    guard = 8; train = 16;
    n_ref = (2*(guard+train)+1)^2 - (2*guard+1)^2;
    pfa = cfg.cfar_pfa;
    alpha_mean = n_ref*(pfa^(-1/n_ref) - 1);
    b_min0 = max(0, floor(cfg.depth_min/dr));
    b_max0 = min(n_ifft-1, floor(cfg.depth_max/dr));
    if strcmp(cfg.cfar_estimator, 'log')
        cfar_mode = 1;
        alpha_log = log(1/pfa);                 % natural log (MATLAB)
        log2alpha = round(log2(alpha_log)*2^26);
    else
        cfar_mode = 0;
        log2alpha = 0;
    end
    gamma2 = round(0.5772156649015329/log(2)*2^26);

    % ---- assemble regs_<tag>.vh ----------------------------------------
    fid = fopen(fullfile(outdir, ['regs_' tag '.vh']), 'w');
    fprintf(fid, '// Auto-generated by make_golden.m - do not edit.\n');
    fprintf(fid, '// Mode %s (%s), F_CLK = %d Hz\n', cfg.mode, cfg.description, F_CLK);
    pr = @(nm, val) fprintf(fid, 'localparam [31:0] %-22s = 32''d%u;\n', nm, val);
    prs = @(nm, val) fprintf(fid, 'localparam signed [31:0] %-22s = 32''sd%d;\n', nm, val);
    pr('GPR_F_CLK_HZ',  F_CLK);
    pr('DWELL_CYC',     round(cfg.tone_dwell_time*F_CLK));
    pr('N_TONES',       cfg.n_tones);
    pr('N_POS',         cfg.n_positions);
    pr('N_IFFT',        cfg.n_ifft);
    pr('LOG2_NAVG',     round(log2(cfg.n_averages)));
    pr('N_AVG',         cfg.n_averages);
    pr('FFT_GAIN_Q10',  18872);                 % floor(18.429835244116052*1024)
    pr('AGC_GAIN_Q14',  16384);
    pr('NBG',           cfg.bg_remove_modes);
    pr('DR_Q20',        round(dr*2^20));
    pr('DR_Q16',        round(dr*2^16));
    pr('INV_DR_Q20',    round(2^20/dr));
    pr('ROFF_Q20',      round(r_off*2^20));
    pr('DX_Q20',        round(dx*2^20));
    pr('KBETA_Q16',     round(kbeta*2^16));
    prs('ALPHA2_Q8',    round(2*alpha_np*2^8));
    pr('MAX_BIN',       max_bin);
    pr('B_MIN0',        b_min0);
    pr('B_MAX0',        b_max0);
    pr('CFAR_MODE',     cfar_mode);
    pr('ALPHA_Q12',     round(alpha_mean*2^12));
    pr('LOG2ALPHA_Q26', log2alpha);
    pr('GAMMA2_Q26',    gamma2);
    pr('OUT_SCALE_Q8',  256);
    pr('N_SB',          n_sb);
    pr('PFLOOR_K',      2874);                  % 1e-14*2^28 ~= 2874/2^28
    pr('PFLOOR_SH',     28);
    fclose(fid);
    printf('regs_%s.vh: dr=%.4g max_bin=%d b=[%d %d] alpha=%.4g mode=%d n_sb=%d\n', ...
        tag, dr, max_bin, b_min0, b_max0, alpha_mean, cfar_mode, n_sb);
end

%% ------------------------------------------------ window ROM (B10a)
% parsed straight out of the baked code_rangeproc script (single source)
scr = code_rangeproc(cfgA);
tok = regexp(scr, 'w = \[([0-9eE\.\+\- ]+)\]', 'tokens', 'once');
wv  = str2num(['[' tok{1} ']']);               %#ok<ST2NM>
assert(numel(wv) == 256, 'window length');
write_hex(fullfile(outdir, 'win.hex'), round(wv*32767), 4);

%% ------------------------------------------------ twiddle ROMs (B10b)
for N = [64, 2048]
    m = (0:N/2-1)';
    w = exp(-1j*2*pi*m/N);
    tw = bitor_shift(round(real(w)*32767), round(imag(w)*32767));
    write_hex(fullfile(outdir, sprintf('twid%d.hex', N)), tw, 8);
end

%% ------------------------------------------------ shared math LUTs
nc = (1:64)';
write_hex(fullfile(outdir, 'invsqrt_nc.hex'), round(2^15./sqrt(nc)), 4);
u = (0:1023)'/128;                             % 0..8 in 1/128 steps
write_hex(fullfile(outdir, 'exp_lut.hex'), round(exp(-u)*32767), 4);
x = (0:63)'/64;
write_hex(fullfile(outdir, 'log2_lut.hex'), round(log2(1 + x)*2^26), 7);
cnt = (1:2401)';
write_hex(fullfile(outdir, 'inv_cnt.hex'), round(2^30./cnt), 8);

%% =========================== testbench vectors =========================
%% FFT golden: 256 random tones -> 2048-point ifft * 18.4298
rng(7);
xin = complex(round((rand(256,1)*2-1)*30000), round((rand(256,1)*2-1)*30000));
write_hex(fullfile(outdir, 'fft_in.hex'), bitor_shift(real(xin), imag(xin)), 8);
X = [xin; zeros(2048-256, 1)];
rpf = ifft(X)*18.429835244116052;
write_hex(fullfile(outdir, 'fft_gold.hex'), bitor_shift(sat16(round(real(rpf))), sat16(round(imag(rpf)))), 8);

%% background golden (small): N_IFFT=64 x N_POS=8, exact integer check
NI = 64; NP = 8;
A = complex(round((rand(NI,NP)*2-1)*30000), round((rand(NI,NP)*2-1)*30000));
write_hex(fullfile(outdir, 'bg_in.hex'), bitor_shift(real(A), imag(A)), 8);
br = median(real(A), 2); bi = median(imag(A), 2);   % even NP: exact mean
br = floor(br); bi = floor(bi);                     % RTL floors the /2
bs = (real(A) - repmat(br,1,NP)) + 1i*(imag(A) - repmat(bi,1,NP));
bsq = sat16(real(bs)) + 1i*sat16(imag(bs));
% RTL streams the frame column (position) by column, depth fastest
fid = fopen(fullfile(outdir, 'bg_gold.hex'), 'w');
for j = 1:NP
    for i = 1:NI
        fprintf(fid, '%s\n', hex32(real(bsq(i,j)), imag(bsq(i,j))));
    end
end
fclose(fid);

%% CFAR golden (small patch, mean estimator): exact integer replication
R = 64; C = 64; GUARD = 4; TRAIN = 8; W = GUARD + TRAIN;
rng(11);
img = uint16(1000 + floor(rand(R, C)*400));
img(20:24, 30:34) = 8000;      % blob 1
img(45:48, 12:15) = 6000;      % blob 2
pm = double(img).^2;                       % <= 2^26, exact in double
pm_max = max(pm(:));
p_floor = floor(pm_max*2874/2^28);
pmf = max(pm, p_floor);
BMIN0 = 4; BMAX0 = 59;
valid = zeros(R, C); valid(BMIN0+1:BMAX0+1, :) = 1;
pmap = pmf .* valid;
kb = ones(2*W+1, 1); kr = ones(1, 2*W+1);
kg = ones(2*GUARD+1, 1); kgr = ones(1, 2*GUARD+1);
big_cnt = conv2(conv2(valid, kb, 'same'), kr, 'same');
g_cnt   = conv2(conv2(valid, kg, 'same'), kgr, 'same');
cnt = big_cnt - g_cnt; cnt(cnt < 1) = 1;
big_sum = conv2(conv2(pmap, kb, 'same'), kr, 'same');
g_sum   = conv2(conv2(pmap, kg, 'same'), kgr, 'same');
n_ref = (2*W+1)^2 - (2*GUARD+1)^2;
pfa = 1e-6;
alpha_q12 = round(n_ref*(pfa^(-1/n_ref) - 1)*2^12);
det = (pmap*2^12 .* cnt > alpha_q12*(big_sum - g_sum)) & (valid > 0) & (cnt > 0);
write_img(fullfile(outdir, 'cfar_in.hex'), img);
fid = fopen(fullfile(outdir, 'cfar_gold.hex'), 'w');
for i = 1:R                       % row-major (depth row by row), like RTL
    for j = 1:C
        fprintf(fid, '%d\n', det(i, j));
    end
end
fclose(fid);
printf('cfar golden: %d detections, alpha_q12=%d\n', sum(det(:)), alpha_q12);

%% clustering golden: replicate pass1/pass2 integer semantics on det
MAXLAB = 256;
L = zeros(R, C); parent = zeros(1, MAXLAB); nlab = 0;
for i = 1:R
    for j = 1:C
        if ~det(i, j), continue; end
        best = 0;
        if i > 1
            if j > 1   && L(i-1,j-1) > 0, [best, parent] = uf(best, L(i-1,j-1), parent); end
            if           L(i-1,j)   > 0, [best, parent] = uf(best, L(i-1,j),   parent); end
            if j < C   && L(i-1,j+1) > 0, [best, parent] = uf(best, L(i-1,j+1), parent); end
        end
        if j > 1 && L(i,j-1) > 0, [best, parent] = uf(best, L(i,j-1), parent); end
        if best == 0
            if nlab < MAXLAB, nlab = nlab + 1; parent(nlab) = nlab; best = nlab;
            else, best = MAXLAB; end
        end
        L(i, j) = best;
    end
end
cntc = zeros(1,MAXLAB); pk = -inf(1,MAXLAB); pkr = zeros(1,MAXLAB); pkc = zeros(1,MAXLAB);
for i = 1:R
    for j = 1:C
        lab = L(i,j); if lab <= 0, continue; end
        r = ufind(lab, parent); L(i,j) = r;
        cntc(r) = cntc(r) + 1;
        if double(img(i,j)) > pk(r), pk(r) = double(img(i,j)); pkr(r) = i; pkc(r) = j; end
    end
end
DR_Q16 = 283; DX_Q16 = round(0.05*2^16);
nclust = 0; sum_d = 0; sum_x = 0; dmin = 0; dmax = 0;
for r = 1:MAXLAB
    if cntc(r) <= 0, continue; end
    nclust = nclust + 1;
    d = (pkr(r) - 1)*DR_Q16; xx = (pkc(r) - 1)*DX_Q16;
    sum_d = sum_d + d; sum_x = sum_x + xx;
    if nclust == 1, dmin = d; dmax = d; end
    if d < dmin, dmin = d; end
    if d > dmax, dmax = d; end
end
mean_d = 0; mean_x = 0;
if nclust > 0, mean_d = floor(sum_d/nclust); mean_x = floor(sum_x/nclust); end
rep = [sum(det(:)), nclust, mean_d, mean_x, dmin, dmax];
fid = fopen(fullfile(outdir, 'cluster_gold.hex'), 'w');
fprintf(fid, '%d\n', rep);
fclose(fid);
printf('cluster golden: rep = %s\n', mat2str(rep));

%% migration golden (small grid, float reference, tolerance check)
NIT = 64; NPT = 16; MAXB = 12; NAPT = 8;
drt = 0.00431; dxt = 0.01; kbt = 5.821; a2t = 1.0796; rofft = 0.0;
rng(23);
dat = complex(round((rand(NIT,NPT)*2-1)*3000), round((rand(NIT,NPT)*2-1)*3000));
fid = fopen(fullfile(outdir, 'mig_in.hex'), 'w');
for j = 1:NPT
    for i = 1:NIT
        fprintf(fid, '%s\n', hex32(real(dat(i,j)), imag(dat(i,j))));
    end
end
fclose(fid);
napg = zeros(MAXB, 1);
mig = zeros(MAXB, NPT);
for iz = 1:MAXB
    z = (iz - 1)*drt;
    if z <= 0, continue; end
    nap = max(1, floor(z/dxt)); nap = min(nap, NAPT);
    napg(iz) = nap;
    for ix = 1:NPT
        acc = 0; nc = 0;
        for s = -nap:nap
            it = ix + s;
            if it < 1 || it > NPT, continue; end
            xs = s*dxt;
            Rr = sqrt(z*z + xs*xs);
            fb = 1 + (Rr + rofft)/drt;
            b0 = floor(fb); fr = fb - b0;
            if b0 < 1 || b0 + 1 > NIT, continue; end
            val = (1 - fr)*dat(b0, it) + fr*dat(b0 + 1, it);
            w = (z/Rr)/sqrt(Rr)*exp(-a2t*(Rr - z));
            w = w*0.5*(1 + cos(pi*s/(nap + 1)));
            acc = acc + w*val*exp(1j*2*pi*kbt*Rr);
            nc = nc + 1;
        end
        if nc > 0, mig(iz, ix) = abs(acc)/sqrt(nc); end
    end
end
fid = fopen(fullfile(outdir, 'mig_gold.hex'), 'w');
for iz = 1:MAXB
    for ix = 1:NPT
        fprintf(fid, '%d\n', round(mig(iz, ix)*1000));
    end
end
fclose(fid);
write_hex_row(fullfile(outdir, 'mig_nap.hex'), napg', 2);

rmdir(fullfile(outdir, 'tmp'), 's');
printf('make_golden: done -> %s\n', outdir);
end

% ------------------------------------------------------------------ utils
function write_hex(path, vals, nchars)
fid = fopen(path, 'w');
for k = 1:numel(vals)
    v = vals(k);
    if v < 0, v = v + 2^(4*nchars); end
    fprintf(fid, ['%0' num2str(nchars) 'x\n'], uint64(v));
end
fclose(fid);
end

function write_hex_row(path, vals, nchars)
fid = fopen(path, 'w');
for k = 1:numel(vals)
    v = vals(k);
    if v < 0, v = v + 2^(4*nchars); end
    fprintf(fid, ['%0' num2str(nchars) 'x\n'], uint64(v));
end
fclose(fid);
end

function write_img(path, img)
fid = fopen(path, 'w');
for i = 1:size(img, 1)
    for j = 1:size(img, 2)
        fprintf(fid, '%04x\n', uint16(img(i, j)));
    end
end
fclose(fid);
end

function v = bitor_shift(re, im)
% pack {re[31:16], im[15:0]} with two's-complement wrap, float-exact
re = mod(round(double(re)), 65536);
im = mod(round(double(im)), 65536);
v = re*65536 + im;
end

function h = hex32(re, im)
h = sprintf('%04x%04x', mod(round(double(re)),65536), mod(round(double(im)),65536));
end

function h = hex16(u)
h = sprintf('%04x', mod(round(double(u)),65536));
end

function y = sat16(x)
y = max(-32768, min(32767, x));
end

function root = ufind(a, parent)
root = a; g = 0;
while root > 0 && parent(root) > 0 && parent(root) ~= root && g < 10000
    root = parent(root); g = g + 1;
end
end

function [merged, parent] = uf(a, b, parent)
if a <= 0, merged = ufind(b, parent); return; end
ra = ufind(a, parent); rb = ufind(b, parent);
if ra == rb, merged = ra; return; end
if ra < rb, parent(rb) = ra; merged = ra;
else, parent(ra) = rb; merged = rb; end
end
