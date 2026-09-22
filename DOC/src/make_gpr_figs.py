#!/usr/bin/env python3
"""Figure engine for the GPM algorithm book and MATLAB help guide.

(S) simulated: a small numpy re-implementation of the GPM chain (tones ->
    two-way channel -> coupling -> noise/averaging -> calibration ->
    Kaiser+zero-pad IFFT -> median background -> Kirchhoff migration ->
    CA-CFAR -> clusters), fixed seeds, captions labelled [simulated].
(D) diagrams: topology/geometry/workflow drawings.
Numbers in the books quote the package itself; figures illustrate behaviour.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
import os

OUT = os.path.join(os.path.dirname(__file__), 'figs_gpr')
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({'font.size': 8.5, 'axes.grid': True, 'grid.alpha': 0.3,
                     'figure.dpi': 130, 'savefig.bbox': 'tight'})
RNG = np.random.default_rng(20260922)
C = 3e8

def save(fig, name):
    fig.savefig(os.path.join(OUT, name)); plt.close(fig); print('fig', name)

def box(ax, x, y, w, h, text, fc='#e8f0fb', fs=7.2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02", fc=fc, ec='#234', lw=0.8))
    ax.text(x + w/2, y + h/2, text, ha='center', va='center', fontsize=fs)

def arrow(ax, x1, y1, x2, y2, text=''):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1), arrowprops=dict(arrowstyle='->', lw=0.9))
    if text: ax.text((x1+x2)/2, (y1+y2)/2+0.03, text, fontsize=6.3, ha='center')

def canvas(w, h, xl, yl):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.axis('off'); ax.grid(False)
    return fig, ax

# ---------------- mini pipeline (mode-A-like, small) ----------------
def mini_chain(n_pos=120, n_tones=128, n_ifft=1024, beta=8, n_avg=32, seed=7):
    rng = np.random.default_rng(seed)
    f = np.linspace(0.5e9, 3.0e9, n_tones)
    x = np.linspace(0, 6, n_pos)
    alt = 0.5; eps_r = 6.0; alpha = 0.6; v = C/np.sqrt(eps_r)
    tgt = [(0.25, 1.5, 0.05), (0.5, 3.0, 0.2), (0.8, 4.5, 0.5)]
    sep = 0.2
    # ripple + cable delay (system response removed by calibration)
    ripple = 1 + 0.15*np.sin(2*np.pi*f/0.9e9)
    delay = 5e-9
    chan = np.zeros((n_tones, n_pos), complex)
    for k in range(n_pos):
        # surface bounce
        R_s = 2*(alt + 0.02)
        chan[:, k] += 0.5*np.exp(-1j*2*np.pi*f*R_s/C)
        for d, xt, rcs in tgt:
            R_air = np.sqrt(alt**2 + (x[k]-xt)**2)
            R_soil = np.sqrt(d**2 + (x[k]-xt)**2*0.25)
            amp = rcs*np.exp(-2*alpha*R_soil)/(R_air+R_soil)**2
            ph = np.exp(-1j*2*np.pi*f*(2*R_air/C + 2*R_soil/v))
            chan[:, k] += amp*ph
    coup = 0.9*np.exp(-1j*2*np.pi*f*sep/C)
    y = (chan + coup[:, None])*ripple[:, None]*np.exp(-1j*2*np.pi*f*delay)[:, None]
    noise_std = 2e-3
    y = y + noise_std/np.sqrt(n_avg)*(rng.normal(size=y.shape)+1j*rng.normal(size=y.shape))
    # calibration divides the known response
    cal = y/(ripple[:, None]*np.exp(-1j*2*np.pi*f*delay)[:, None])
    return f, x, cal, tgt, n_ifft, beta

def range_profiles(cal, f, n_ifft, beta):
    n_t = cal.shape[0]
    w = np.kaiser(n_t, beta)
    rp = np.fft.ifft(cal*w[:, None], n_ifft, axis=0)
    return rp

def migrate(rp, f, x, n_ifft, beta, zmax=1.2, nz=90, nx=90, v=1.22e8):
    z = np.linspace(0.02, zmax, nz); xi = np.linspace(x[0], x[-1], nx)
    img = np.zeros((nz, nx))
    n_t = f.size
    t_ax = np.arange(n_ifft)/ (n_ifft) * n_t/ (f[1]-f[0])   # two-way time axis
    for i, zz in enumerate(z):
        for j, xx in enumerate(xi):
            R = np.sqrt(zz**2 + (x-xx)**2)
            t = 2*R/v
            idx = np.interp(t, t_ax, np.arange(n_ifft))
            ok = idx < n_ifft-1
            i0 = np.floor(idx[ok]).astype(int); fr = idx[ok]-i0
            samp = (1-fr)*np.abs(rp[i0, ok]) + fr*np.abs(rp[i0+1, ok])
            img[i, j] = samp.mean()
    return z, xi, img

def cfar2d(img, g=4, tr=8, pfa_mult=6.0):
    nz, nx = img.shape
    p = img**2
    det = np.zeros_like(p, bool)
    for i in range(g+tr, nz-g-tr):
        for j in range(g+tr, nx-g-tr):
            win = p[max(i-g-tr,0):i+g+tr+1, max(j-g-tr,0):j+g+tr+1]
            grd = p[max(i-g,0):i+g+1, max(j-g,0):j+g+1]
            train = (win.sum()-grd.sum())/max(win.size-grd.size,1)
            det[i, j] = p[i, j] > pfa_mult*train
    return det

# ---------------- figures ----------------
def fig_pipeline():
    fig, ax = canvas(11.5, 4.6, (0, 11.5), (0, 4.6))
    rf = ['B01\nEnvironment', 'B02\nWaveform', 'B03\nTX chain', 'B04\nChannel',
          'B05\nCoupling', 'B06\nRX', 'B07\nADC']
    dsp = ['B08\nAveraging', 'B09\nCalibration', 'B10\nRangeProc', 'B11\nBackground',
           'B12\nMigration', 'B13\nDetection', 'B14\nReport']
    for i, b in enumerate(rf):
        box(ax, 0.15+i*1.62, 3.0, 1.45, 0.9, b, fc='#e8f0fb')
        ax.text(0.15+i*1.62+0.72, 2.75, f'TP{i+1:02d}', fontsize=6.3, ha='center', color='#a33')
        if i: arrow(ax, 0.15+(i-1)*1.62+1.45, 3.45, 0.15+i*1.62, 3.45)
    for i, b in enumerate(dsp):
        box(ax, 0.15+i*1.62, 1.0, 1.45, 0.9, b, fc='#dff0df')
        ax.text(0.15+i*1.62+0.72, 0.75, f'TP{i+8:02d}', fontsize=6.3, ha='center', color='#a33')
        if i: arrow(ax, 0.15+(i-1)*1.62+1.45, 1.45, 0.15+i*1.62, 1.45)
    arrow(ax, 0.15+6*1.62+0.72, 3.0, 0.15+0.72, 1.9, 'iq + noise_std')
    ax.text(0.2, 4.3, 'RF_Front_End subsystem', fontsize=8, weight='bold')
    ax.text(0.2, 2.3, 'Digital_Processing subsystem', fontsize=8, weight='bold')
    ax.text(0.2, 0.25, 'TP01-TP14: Scope + To Workspace logger per block output, both engines',
            fontsize=7, style='italic')
    save(fig, 'gpr_pipeline.png')

def fig_geometry():
    fig, ax = plt.subplots(figsize=(8, 4))
    xs = np.linspace(0, 6, 100)
    ax.fill_between(xs, -1.4, 0, color='#d9c8a9', zorder=0)
    ax.plot(xs, np.zeros_like(xs), 'k', lw=1)
    ax.plot(xs, 0.5+0*xs, '--', color='#46a', lw=1)
    ax.plot(xs, 0.5+0.02*np.sin(xs*3), color='#46a', lw=2, label='UAV track, standoff 0.5 m')
    for d, xt, r in [(0.25, 1.5, 4), (0.5, 3.0, 6), (0.8, 4.5, 8)]:
        ax.plot(xt, -d, 's', color='r', ms=r)
        ax.plot([xt-0.5, xt, xt+0.5], [-d+0.18, -d, -d+0.18], ':', color='r', lw=0.8)
    ax.annotate('', xy=(1.5, -0.25), xytext=(1.5, 0.5), arrowprops=dict(arrowstyle='<->', color='g'))
    ax.text(1.6, 0.1, '2R slant path', color='g', fontsize=7)
    ax.set_xlabel('cross-range x [m]'); ax.set_ylabel('depth [m]')
    ax.set_title('Survey geometry: n_positions traces along scan_length, targets at (x, depth)')
    ax.legend(fontsize=7)
    save(fig, 'gpr_geometry.png')

def fig_tones():
    f, x, cal, tgt, n_ifft, beta = mini_chain()
    n_t = f.size
    t = np.arange(n_t)*50e-6
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.2))
    axs[0].stem(t*1e3, np.ones(n_t), basefmt=' ', markerfmt='.', linefmt='-', )
    axs[0].set_xlabel('time [ms]'); axs[0].set_title('SFCW tone train: one complex tone per dwell')
    tap = 0.10
    n_tap = int(tap*n_t)
    env = np.ones(n_t)
    if n_tap:
        r = 0.5*(1-np.cos(np.pi*np.arange(n_tap)/n_tap))
        env[:n_tap] = r; env[-n_tap:] = r[::-1]
    axs[1].plot(f/1e9, np.ones(n_t), '.', ms=3, label='flat comb (default)')
    axs[1].plot(f/1e9, env, '.', ms=3, label='raised-cosine edge taper 10%')
    axs[1].set_xlabel('frequency [GHz]'); axs[1].legend(fontsize=7)
    axs[1].set_title('Transmit spectrum amplitude per tone')
    save(fig, 'gpr_tones.png')

def fig_soil():
    f = np.logspace(6.5, 9.5, 60)
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.4))
    for mv, c in [(0.02, 'g'), (0.08, 'b'), (0.2, 'r')]:
        eps_r = 3.0 + 18*mv*(1+0.2*np.log10(f/1e8))      # illustrative Dobson-type trend
        alpha = 0.05 + 55*mv*np.sqrt(f/1e9) + 1.0*np.sqrt(f/1e9)
        axs[0].semilogx(f/1e6, eps_r, c, lw=1.2, label=f'mv={mv}')
        axs[1].loglog(f/1e6, alpha, c, lw=1.2)
    axs[0].set_xlabel('MHz'); axs[0].set_ylabel('eps_r'); axs[0].legend(fontsize=7)
    axs[0].set_title('Real permittivity vs moisture (Dobson-type trend)')
    axs[1].set_xlabel('MHz'); axs[1].set_ylabel('alpha [Np/m]')
    axs[1].set_title('Attenuation: why mode B goes to 10-100 MHz')
    save(fig, 'gpr_soil.png')

def fig_ascan():
    f, x, cal, tgt, n_ifft, beta = mini_chain()
    rp = range_profiles(cal, f, n_ifft, beta)
    k = 60
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    d = np.arange(n_ifft)/n_ifft*(f.size/(f[1]-f[0]))*1.22e8/2
    ax.plot(20*np.log10(np.abs(rp[:, k])+1e-9), lw=1.0)
    ax.set_xlabel('range bin (zero-padded IFFT)'); ax.set_ylabel('dB')
    ax.set_title('One A-scan: coupling + surface + three target echoes [simulated]')
    save(fig, 'gpr_ascan.png')

def fig_window():
    n = 128
    wr = np.ones(n); wk = np.kaiser(n, 8)
    N = 1024
    FR = np.abs(np.fft.ifft(wr, N)); FK = np.abs(np.fft.ifft(wk, N))
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.3))
    axs[0].plot(wr, label='rectangular'); axs[0].plot(wk, label='Kaiser beta=8')
    axs[0].legend(fontsize=7); axs[0].set_title('Spectral taper w[n]')
    axs[1].plot(20*np.log10(FR/FR.max()+1e-12), label='rect: -13 dB sidelobes')
    axs[1].plot(20*np.log10(FK/FK.max()+1e-12), label='Kaiser: ~-58 dB')
    axs[1].set_ylim(-90, 5); axs[1].legend(fontsize=7)
    axs[1].set_title('Compressed pulse sidelobes')
    save(fig, 'gpr_window.png')

def fig_rangecomp():
    f, x, cal, tgt, n_ifft, beta = mini_chain()
    rp_k = range_profiles(cal, f, n_ifft, beta)
    rp_r = range_profiles(cal, f, n_ifft, 0.01)
    fig, ax = plt.subplots(figsize=(7.5, 3.5))
    ax.plot(20*np.log10(np.abs(rp_r[:, 60])+1e-9), lw=0.9, label='rectangular: sidelobe skirt')
    ax.plot(20*np.log10(np.abs(rp_k[:, 60])+1e-9), lw=0.9, label='Kaiser beta=8')
    ax.set_ylim(-120, 0); ax.legend(fontsize=7)
    ax.set_xlabel('range bin'); ax.set_ylabel('dB')
    ax.set_title('Range compression: the skirt that migration would sum coherently [simulated]')
    save(fig, 'gpr_rangecomp.png')

def fig_noise():
    items = [('thermal kTB (50 us dwell, NF 4 dB)', -104), ('RX cable loss + LNA gain', 6),
             ('ADC SQNR 6.02*14+1.76', 86), ('coherent avg /sqrt(32)', 15)]
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    cum = 0
    for i, (n, v) in enumerate(items):
        ax.bar(i, abs(v), bottom=min(cum, cum+v) if v < 0 else cum, color='#7ab' if v > 0 else '#fa7')
        cum += v
        ax.text(i, cum+2, f'{cum:.0f}', ha='center', fontsize=7)
    ax.set_xticks(range(len(items))); ax.set_xticklabels([n for n, _ in items], rotation=18, ha='right', fontsize=6.6)
    ax.set_ylabel('dBm / dB'); ax.set_title('Noise budget shape: thermal floor, SQNR headroom, sqrt(N) gain [derived]')
    save(fig, 'gpr_noise.png')

def fig_gain():
    N = np.arange(1, 129)
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.loglog(N, np.sqrt(N), 'k', lw=1.4, label='sqrt(N) amplitude gain (correct)')
    ax.loglog(N, N, '--', color='r', lw=1.0, label='N (the old bug: noise /N)')
    ax.legend(fontsize=7); ax.set_xlabel('n_averages'); ax.set_ylabel('gain')
    ax.set_title('Coherent integration gain: sqrt(N), never N')
    save(fig, 'gpr_gain.png')

def fig_cal():
    f, x, cal, tgt, n_ifft, beta = mini_chain()
    ripple = 1+0.15*np.sin(2*np.pi*f/0.9e9)
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    ax.plot(f/1e9, 20*np.log10(ripple), label='system ripple (cable + passband)')
    ax.plot(f/1e9, 20*np.log10(np.abs(cal[:, 60])/np.abs(cal[:, 60]).mean()+1e-9)-0, lw=0.8,
            label='calibrated trace spectrum (ripple divided out)')
    ax.legend(fontsize=7); ax.set_xlabel('GHz'); ax.set_ylabel('dB')
    ax.set_title('B09 calibration divides the known transfer function [simulated]')
    save(fig, 'gpr_cal.png')

def fig_bscan_trio():
    f, x, cal, tgt, n_ifft, beta = mini_chain()
    rp = range_profiles(cal, f, n_ifft, beta)
    mag = np.abs(rp[:400, :])
    bg = np.median(mag, axis=1, keepdims=True)
    clean = mag - bg
    fig, axs = plt.subplots(1, 3, figsize=(10.5, 3.4), sharey=True)
    for ax, m, t in zip(axs, [mag, np.repeat(bg, mag.shape[1], axis=1), clean],
                        ['raw B-scan (coupling band + hyperbolas)',
                         'component-wise median background',
                         'background removed: hyperbolas remain']):
        ax.pcolormesh(20*np.log10(m/m.max()+1e-6), cmap='gray', shading='auto')
        ax.set_title(t, fontsize=7.5); ax.set_xlabel('position')
    axs[0].set_ylabel('range bin')
    save(fig, 'gpr_bscan_trio.png')

def fig_hyper():
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    xs = np.linspace(0, 6, 200)
    for d, xt in [(0.25, 1.5), (0.5, 3.0), (0.8, 4.5)]:
        R = np.sqrt(d**2+(xs-xt)**2)
        ax.plot(xs, -R, lw=1.1)
        ax.plot(xt, -d, 'rs', ms=6)
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xlabel('cross-range x [m]'); ax.set_ylabel('apparent depth [m]')
    ax.set_title('Diffraction hyperbolae R = sqrt(z^2 + (x-xt)^2): what migration collapses')
    save(fig, 'gpr_hyper.png')

def fig_migrated():
    f, x, cal, tgt, n_ifft, beta = mini_chain()
    rp = range_profiles(cal, f, n_ifft, beta)
    mag = np.abs(rp)
    clean = mag - np.median(mag, axis=1, keepdims=True)
    z, xi, img = migrate(clean, f, x, n_ifft, beta)
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    axs[0].pcolormesh(x, np.arange(clean.shape[0])[:400]*0.003, 20*np.log10(clean[:400]/clean[:400].max()+1e-6),
                      cmap='gray', shading='auto')
    axs[0].set_title('unmigrated (range bin vs position)')
    axs[1].pcolormesh(xi, z, 20*np.log10(img/img.max()+1e-6), cmap='gray', shading='auto')
    for d, xt, r in tgt: axs[1].plot(xt, d, 'r+', ms=9)
    axs[1].set_title('Kirchhoff-migrated image, truths marked')
    axs[1].invert_yaxis(); axs[0].set_ylabel('depth / range')
    axs[1].set_xlabel('x [m]')
    save(fig, 'gpr_migrated.png')
    return z, xi, img

def fig_cfar(img=None):
    if img is None:
        f, x, cal, tgt, n_ifft, beta = mini_chain()
        rp = range_profiles(cal, f, n_ifft, beta)
        mag = np.abs(rp); clean = mag-np.median(mag, axis=1, keepdims=True)
        z, xi, img = migrate(clean, f, x, n_ifft, beta)
    det = cfar2d(img)
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
    axs[0].pcolormesh(20*np.log10(img/img.max()+1e-6), cmap='inferno', shading='auto')
    yy, xx = np.where(det)
    axs[0].plot(xx, yy, 'c.', ms=4)
    axs[0].set_title(f'migrated image + CA-CFAR detections ({det.sum()} cells)')
    cut = img[:, img.shape[1]//2]**2
    axs[1].plot(10*np.log10(cut+1e-12), lw=0.9, label='cell power')
    axs[1].plot(10*np.log10(np.convolve(cut, np.ones(17)/17, 'same')*6+1e-12), 'r', lw=0.9, label='threshold (train mean x alpha)')
    axs[1].legend(fontsize=7); axs[1].set_xlabel('depth bin'); axs[1].set_ylabel('dB')
    axs[1].set_title('CFAR cut at mid-profile')
    save(fig, 'gpr_cfar.png')
    return det

def fig_clusters(det=None):
    if det is None:
        det = np.zeros((60, 60), bool)
        det[20:23, 14:17] = True; det[35:38, 30:34] = True; det[45:47, 44:46] = True
        det[10, 50] = True
    fig, ax = plt.subplots(figsize=(6.5, 4))
    lab = np.zeros(det.shape, int)
    # simple 8-connected labelling
    from collections import deque
    nl = 0
    for i in range(det.shape[0]):
        for j in range(det.shape[1]):
            if det[i, j] and lab[i, j] == 0:
                nl += 1; q = deque([(i, j)]); lab[i, j] = nl
                while q:
                    a, b = q.popleft()
                    for da in (-1, 0, 1):
                        for db in (-1, 0, 1):
                            na, nb = a+da, b+db
                            if 0 <= na < det.shape[0] and 0 <= nb < det.shape[1] and det[na, nb] and lab[na, nb] == 0:
                                lab[na, nb] = nl; q.append((na, nb))
    ax.pcolormesh(lab, cmap='tab10', shading='auto')
    ax.set_title(f'8-connected clustering: {nl} distinct detections from {det.sum()} cells')
    ax.set_xlabel('x bin'); ax.set_ylabel('depth bin')
    save(fig, 'gpr_clusters.png')

def fig_modeAB():
    fig, axs = plt.subplots(1, 3, figsize=(10, 3.2))
    axs[0].bar(['A', 'B'], [2.5e9, 90e6], color=['#7ab', '#fa7'])
    axs[0].set_yscale('log'); axs[0].set_ylabel('Hz'); axs[0].set_title('bandwidth')
    axs[1].bar(['A', 'B'], [1.0, 18.0], color=['#7ab', '#fa7'])
    axs[1].set_ylabel('m'); axs[1].set_title('depth window')
    axs[2].bar(['A', 'B'], [3e8/(2*2.5e9)*np.sqrt(6), 3e8/(2*90e6)*np.sqrt(6)/np.sqrt(6)*1], color=['#7ab', '#fa7'])
    axs[2].set_yscale('log'); axs[2].set_ylabel('m (in-soil, illustrative)'); axs[2].set_title('range cell')
    save(fig, 'gpr_modeAB.png')

def fig_enob():
    enob = np.arange(6, 25)
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(enob, 6.02*enob+1.76, lw=1.4)
    ax.plot(14, 6.02*14+1.76, 'ro'); ax.text(14.3, 6.02*14+1.76, 'mode A: 14 bit', fontsize=7)
    ax.plot(16, 6.02*16+1.76, 'go'); ax.text(16.3, 6.02*16+1.76, 'mode B: 16 bit', fontsize=7)
    ax.set_xlabel('ENOB'); ax.set_ylabel('SQNR [dB]')
    ax.set_title('Full-scale-sine law SQNR = 6.02*ENOB + 1.76 dB (v1.7 fix)')
    save(fig, 'gpr_enob.png')

def fig_engines():
    fig, ax = canvas(10.5, 3.4, (0, 10.5), (0, 3.4))
    box(ax, 3.9, 2.3, 2.6, 0.8, 'gpr_pipeline_spec.m\nsingle source of truth', fc='#fdf6dd')
    box(ax, 0.4, 0.7, 3.0, 0.9, 'build_realistic_model.m\n-> Simulink MATLAB Function\nblocks + TP scopes/loggers', fc='#e8f0fb')
    box(ax, 6.9, 0.7, 3.2, 0.9, 'run_pipeline_reference.m\n-> gpr_compile_script: same text\nas temp functions, MATLAB/Octave', fc='#dff0df')
    arrow(ax, 4.6, 2.3, 2.4, 1.6); arrow(ax, 5.9, 2.3, 8.0, 1.6)
    ax.text(0.4, 0.15, 'both engines log TP01-TP14; gpr_extract_results gives one results struct', fontsize=7, style='italic')
    save(fig, 'gpr_engines.png')

def fig_tpmap():
    fig, ax = canvas(10.5, 3.0, (0, 10.5), (0, 3))
    for i in range(14):
        col = '#e8f0fb' if i < 7 else '#dff0df'
        ax.add_patch(Rectangle((0.2+i*0.72, 1.4), 0.66, 0.8, fc=col, ec='#234', lw=0.7))
        ax.text(0.2+i*0.72+0.33, 1.8, f'TP{i+1:02d}', ha='center', fontsize=6.5)
    ax.text(0.2, 0.9, 'Simulink: TPnn_Scope (visible Scope, one input per port) + tpnn_<block> To Workspace timeseries', fontsize=7)
    ax.text(0.2, 0.45, 'reference engine: same names as struct fields; tpNN_..._o2 for second output ports', fontsize=7)
    save(fig, 'gpr_tpmap.png')

def fig_path():
    fig, ax = canvas(9.5, 3.0, (0, 9.5), (0, 3))
    box(ax, 0.3, 1.6, 2.4, 0.8, 'MATLAB path entry 1:\nOLD unzip of GPM v1.5', fc='#fde8e8')
    box(ax, 0.3, 0.5, 2.4, 0.8, 'MATLAB path entry 2:\nTHIS installation v1.8', fc='#dff0df')
    box(ax, 4.0, 1.0, 2.4, 0.8, 'MATLAB resolver:\nfirst match wins', fc='#fdf6dd')
    box(ax, 7.2, 1.0, 2.0, 0.8, 'mixed-version\npipeline: unrepro-\nducible errors', fc='#fde8e8')
    arrow(ax, 2.7, 2.0, 4.0, 1.6); arrow(ax, 2.7, 0.9, 4.0, 1.2); arrow(ax, 6.4, 1.4, 7.2, 1.4)
    ax.text(0.3, 0.05, 'gpr_path_sanity() lists core functions that do NOT resolve into this folder', fontsize=7, style='italic')
    save(fig, 'gpr_path.png')

def fig_workflow():
    fig, ax = canvas(10.5, 3.2, (0, 10.5), (0, 3.2))
    st = ['config_mode_A/B\n(cfg struct)', 'gpr_check_config\n(validate + derive)',
          'gpr_realistic_main\n(engine choice)', 'gpr_extract_results\n(res struct)',
          'gpr_realistic_gui\n(B-scan, detection,\nprofile)']
    x = 0.2
    for s in st:
        box(ax, x, 1.3, 1.85, 1.0, s); x += 2.04
    for i in range(4):
        arrow(ax, 0.2+1.85+i*2.04, 1.8, 0.2+2.04+i*2.04, 1.8)
    ax.text(0.2, 0.6, 'validate_package() and test_gpr_package() gate any change before it is trusted', fontsize=7.5, style='italic')
    save(fig, 'gpr_workflow.png')

def fig_dispersion():
    f = np.linspace(0.5e9, 3e9, 200)
    ph_nd = -2*np.pi*f*1e-8
    ph_d = ph_nd*(1+0.06*np.sqrt(f/3e9))
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(f/1e9, np.unwrap(ph_nd)/np.pi, label='non-dispersive: phase linear in f')
    ax.plot(f/1e9, np.unwrap(ph_d)/np.pi, label='dispersive (NOT used by B04)')
    ax.legend(fontsize=7); ax.set_xlabel('GHz'); ax.set_ylabel('phase [pi]')
    ax.set_title('B04 soil model is non-dispersive: phase slope = travel time')
    save(fig, 'gpr_dispersion.png')

if __name__ == '__main__':
    fig_pipeline(); fig_geometry(); fig_tones(); fig_soil(); fig_ascan()
    fig_window(); fig_rangecomp(); fig_noise(); fig_gain(); fig_cal()
    fig_bscan_trio(); fig_hyper(); z, xi, img = fig_migrated(); det = fig_cfar(img)
    fig_clusters(det); fig_modeAB(); fig_enob(); fig_engines(); fig_tpmap()
    fig_path(); fig_workflow(); fig_dispersion()
    import glob
    print('total:', len(glob.glob(os.path.join(OUT, '*.png'))))
