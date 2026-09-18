function gpr_realistic_main()
% GPR_REALISTIC_MAIN  Physics-aware GPR digital twin entry point.
% The complete RF front end and digital processing chain are built inside
% Simulink. Every algorithm block receives an inspection logger and Scope.

clc;
fprintf('GPR DIGITAL TWIN: visible RF front end + inspectable DSP chain\n\n');
mode = upper(input('Select mode [A=UAV shallow, B=ground deep] (default B): ', 's'));
if isempty(mode), mode = 'B'; end
if ~ismember(mode, {'A','B'})
    error('Invalid mode. Choose A or B.');
end

if mode == 'A'
    cfg = config_mode_A_uav_shallow();
else
    cfg = config_mode_B_ground_deep();
end

fprintf('\n[1/4] Physics feasibility check...\n');
feasibility = check_physics_feasibility(cfg);
if ~feasibility.viable
    fprintf('\nConfiguration warning: %s\n', feasibility.reason);
    for k = 1:numel(feasibility.suggestions)
        fprintf('  - %s\n', feasibility.suggestions{k});
    end
    proceed = input('Build the model anyway? [y/N]: ', 's');
    if ~strcmpi(proceed,'y'), return; end
end

fprintf('[2/4] Building visible Simulink model...\n');
model_name = build_realistic_model(cfg);

fprintf('[3/4] Compiling model...\n');
set_param(model_name, 'SimulationCommand', 'update');
fprintf('[4/4] Running simulation...\n');
sim_out = sim(model_name);

% The GUI is optional: the model and every test-point variable remain
% available even if the user closes the GUI.
gpr_realistic_gui(model_name, cfg, feasibility, sim_out);
fprintf('Done. Inspect TPxx Scopes or tp_* workspace variables in Simulink.\n');
end
