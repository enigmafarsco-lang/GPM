function v = gpr_cfg_value(cfg, field, default)
% GPR_CFG_VALUE  Read an optional field from a configuration struct.
%
%   V = GPR_CFG_VALUE(cfg, 'field', default) returns cfg.field when it
%   exists and is non-empty, otherwise default. It keeps the code generators
%   backwards compatible with configuration files that predate a field.

if isstruct(cfg) && isfield(cfg, field) && ~isempty(cfg.(field))
    v = cfg.(field);
else
    v = default;
end
end
