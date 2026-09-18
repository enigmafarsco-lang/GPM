function validate_package()
% Basic source-level validation before opening Simulink.
required={'gpr_realistic_main.m','config_mode_A_uav_shallow.m', ...
    'config_mode_B_ground_deep.m','check_physics_feasibility.m', ...
    'build_realistic_model.m','code_generators.m','gpr_realistic_gui.m'};
for k=1:numel(required)
    assert(exist(required{k},'file')==2,['Missing file: ' required{k}]);
end
assert(exist('Simulink.Parameter','class')==8 || exist('Simulink.Parameter','file')==2, ...
    'Simulink is required.');
which('gpr_realistic_main')
fprintf('Source package validation passed.\n');
end
