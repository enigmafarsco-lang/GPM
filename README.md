# GPM — GPR digital twin (Simulink + reference engine)

A scripted, fully parameterised digital twin of a stepped-frequency
ground-penetrating radar: RF front end (environment → waveform → TX chain →
channel → coupling → RX → ADC) and digital processing (averaging →
calibration → range processing → background removal → migration → CFAR
detection → report), built as a Simulink model **and** executable without
Simulink.

Both execution paths are generated from one source of truth,
`gpr_pipeline_spec.m`, so the model and the reference runner cannot drift
apart. Every algorithm block B01–B14 is a generated MATLAB script
(`code_*.m`); `build_realistic_model.m` pastes each script into a Simulink
MATLAB Function block, `run_pipeline_reference.m` compiles the same text to
a temporary function and runs the chain in plain MATLAB/Octave. Test points
TP01–TP14 log every block output in both engines; in Simulink each TP is a
visible Scope (`TPnn_Scope`, one input per output port) plus a To Workspace
timeseries logger (`tpnn_<block>`, `..._o2` for second ports).

```
RF_Front_End                     Digital_Processing
B01_Environment   TP01           B08_Averaging    TP08
B02_Waveform      TP02           B09_Calibration  TP09
B03_TX_Chain      TP03           B10_RangeProc    TP10
B04_Channel       TP04           B11_Background   TP11
B05_Coupling      TP05           B12_Migration    TP12
B06_RX            TP06 (+noise)  B13_Detection    TP13
B07_ADC           TP07 (+noise)  B14_Report       TP14  -> Final_Report_Log
```

## Quick start

```matlab
cd GPM                      % the folder you cloned or unzipped
addpath(pwd);               % put it on the MATLAB path
validate_package            % static checks, no Simulink needed
test_gpr_package            % functional suite (~2 min, both modes)

res = gpr_realistic_main('mode', 'A');   % UAV-mounted, 0.5-3 GHz, shallow
res = gpr_realistic_main('mode', 'B');   % ground-coupled, 10-100 MHz, deep
gpr_realistic_gui(res)                   % B-scan, detections, report, audit
```

Without Simulink installed, `gpr_realistic_main` automatically uses the
reference engine (`'engine','reference'` forces it, `'engine','simulink'`
forces the model). With Simulink, the model `GPR_Realistic.slx` is built and
updated once; open it to inspect `RF_Front_End`, `Digital_Processing`, the
TP01–TP14 scopes and the `tp01_b01_environment … tp14_b14_report`
To Workspace logs (`gpr_tp_from_simout` collects them back into the `tp`
struct that `gpr_extract_results` understands).

## What the two modes demonstrate

| | Mode A `A_UAV_Shallow` | Mode B `B_Ground_Deep` |
|---|---|---|
| platform | UAV, 0.5 m standoff | ground-coupled, 2 cm |
| band | 0.5–3 GHz, 256 tones | 10–100 MHz, 512 tones |
| survey | 200 traces / 10 m | 200 traces / 20 m |
| soil | εr 3.05, 4.7 dB/m | εr 3.00, 0.9 dB/m |
| targets | 4 (0.1–0.8 m) | 5 (3–15 m) |
| CFAR | mean (classical CA-CFAR) | log (geometric-mean, robust) |
| result | **4/4 detected, 0 false**, ≤2 cm error | **5/5 detected, 0 false**, ≤12 cm error |

Reference-engine timings on a laptop-class CPU: 17 s (A), 90 s (B).
Figures produced by `gpr_realistic_gui` are in `figures/`.

The estimator choice is physics, not taste: in mode A the Kaiser-windowed
range sidelobe skirt sits ~10 dB above the noise floor, and a log-domain
estimator flags that skirt; the arithmetic mean is inflated by the skirt and
suppresses it. In mode B the deep targets sit inside each other's migration
footprints, an arithmetic mean is inflated by neighbouring blobs and masks
targets 3–5; the geometric mean is not. `config_mode_*.m` carry the
rationale in comments, and `gpr_check_config` enforces the sampling rule
that makes migration valid (≥1 trace per Fresnel zone at the shallowest
target — mode A needs ≥~145 traces, the shipped 200 give 1.39).

## Package layout

| file | role |
|---|---|
| `gpr_pipeline_spec.m` | single source of truth: blocks, wiring, parameters |
| `code_*.m` (14) | the algorithm, one function per block, returns its script |
| `build_realistic_model.m` | builds + updates `GPR_Realistic.slx` from the spec |
| `run_pipeline_reference.m` | runs the same scripts without Simulink |
| `gpr_tp_from_simout.m` | SimulationOutput → `tp` struct |
| `gpr_extract_results.m` | `tp` → detections, clusters, report, axes |
| `gpr_check_config.m` | validates a configuration, derives all derived fields |
| `check_physics_feasibility.m` | independent audit: soil, link budget, noise, dynamic range, timing, CFAR expectation |
| `gpr_realistic_main.m` | end-to-end driver (config → audit → engine → score) |
| `gpr_realistic_gui.m` | result viewer (classic HG, works in Octave too) |
| `validate_package.m` | 77 static checks (files, signatures, wiring, physics) |
| `test_gpr_package.m` | 12 functional tests incl. both end-to-end modes |
| `config_mode_*.m` | the two shipped configurations |
| `soil_permittivity.m`, `gpr_axes.m`, `gpr_pulse_width.m`, `gpr_range_window.m` | shared physics/axes helpers |
| `gpr_parse_script.m`, `gpr_compile_script.m`, `gpr_cfg_value.m` | generator plumbing |

## Corrections relative to the original upload

1. `code_generators.m` (one file, 14 functions whose names never matched the
   file name, so MATLAB resolved none of them) is gone; each block has its
   own `code_*.m` whose first function name equals the file name —
   `validate_package` checks this for all 14.
2. `create_all_params` built its parameter list as a 1×38 row cell, so the
   install loop created exactly one parameter. Parameters now come from
   `spec.params`, one `{name, value}` pair per row.
3. Constant blocks used the non-existent library path
   `simulink/Ports & Subsystems/Constant`; they now come from
   `simulink/Sources/Constant`.
4. The model called `set_param(...,'update')` *before* assigning each
   MATLAB Function block's script, which errors out; the diagram is updated
   exactly once, after everything is wired.
5. Broad `try/catch` blocks that swallowed wiring errors are gone; a broken
   link now fails loudly.
6. The GUI referenced `feasibility.viable/.range_res/...` (fields that never
   existed), shadowed the `grid` function with a variable named `grid`, and
   plotted no results. It now renders the migrated B-scan with truth and
   detections overlaid, the detection mask, a depth profile, the B14 report
   and the full physics audit.
7. Simulink size inference is no longer left to chance: `gpr_block_io_sizes.m`
   derives every port size/complexity analytically from the configuration and
   `build_realistic_model.m` stamps them onto the Stateflow chart data, which
   is what R2024a asks for when it reports "not enough information to
   determine output sizes for this block".
8. TP01–TP14 now carry visible Scope blocks in addition to the To Workspace
   loggers, as the model documentation always promised.
9. `gpr_check_config` rejects a non-power-of-two `n_ifft` explicitly (the test
   suite asserted the rejection before the rule existed).
10. `B10_RangeProc` kept its Kaiser window in a `persistent` cache; MATLAB
    Coder rejects any read of a persistent variable that is not an isempty
    guard, which surfaced as "underspecified signal dimensions" for the whole
    model. The window is now a numeric literal baked in at generation time
    (bit-identical output), and `validate_package` fails if any generated
    script ever reintroduces a persistent.
11. Algorithm fixes found by actually running the chain: IFFT dimension in
   B10, cable-delay/air-leg consistency in B09, component-wise median
   background in B11 (mean/SVD smeared targets), coherent
   `exp(+j2βR)` migration stack in B12 (envelope migration defocused deep
   targets), dispersion evaluated at `f_center`, valid-band masking in the
   CFAR, and an exact two-pass union-find cluster count in B14 (the old
   seed rule over-counted 8-connected blobs 29 vs 5).

## Shipping a compiled .slx (optional)

`GPR_Realistic.slx` is a build artefact and is git-ignored on purpose: the
binary is tied to the Simulink release that wrote it, while the scripted
build reproduces it deterministically anywhere (`gpr_realistic_main` or
`build_realistic_model` directly). If you want the binary in the repo anyway,
build it once in your release and `git add -f GPR_Realistic.slx`.

## Known limitations

* The soil model's imaginary part is conduction loss plus a floor; Debye
  relaxation loss of the water content is not modelled (see the note in
  `soil_permittivity.m`), so attenuation grows only weakly with moisture.
* Migration is a straight-ray Kirchhoff stack over a homogeneous half-space;
  the surface bounce is a plane-interface Fresnel term without 1/R²
  spreading (a point-target term diverges at 2 cm standoff).
* The Simulink path was validated by construction and by static checks in a
  Simulink-less environment; the numbers above come from the reference
  engine, which executes byte-identical block code.

## Tests

`test_gpr_package` covers soil physics, axis scaling, config guards,
waveform, cable-delay calibration, background removal, migration focus,
CFAR false-alarm statistics on synthetic noise, 8-connected cluster
labelling, TP naming, and both end-to-end modes (4/4 and 5/5 targets, zero
false alarms). `validate_package` adds 77 static checks. Last full run:
**12/12 tests and 77/77 checks passed.**
