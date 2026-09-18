# GPR Simulink package

This package keeps the complete GPR algorithm inside Simulink.

## Model structure

- `RF_Front_End`: `B01_Environment` through `B07_ADC`
- `Digital_Processing`: `B08_Averaging` through `B14_Report`
- `TP01` through `TP14`: one `Scope` and one `To Workspace` logger after every algorithm block

The package is based on the supplied two-mode design. The main corrections are:

1. The RF front end is a real visible Simulink subsystem, not an implied MATLAB-only step.
2. The digital chain is a separate visible subsystem.
3. Wiring errors are not silently swallowed by broad `try/catch` blocks.
4. Ground-coupled feasibility uses antenna separation as the near-field path scale instead of treating it as UAV altitude.
5. The GUI creates its controls only after `res_axes` exists, so the callback does not reference an uninitialized variable.

## Run

Add this folder to the MATLAB path and run:

```matlab
gpr_realistic_main
```

Open `GPR_Realistic.slx` after the build. Run the model and inspect `TP01` to `TP14`, or the `tp*` variables in the MATLAB workspace.

The package requires MATLAB, Simulink, and Stateflow for the MATLAB Function blocks. It was assembled here but not executed in MATLAB, so run the model update once and fix any version-specific library-path differences in your installed release.
