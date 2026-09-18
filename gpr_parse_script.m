function info = gpr_parse_script(script)
% GPR_PARSE_SCRIPT  Read the port signature of a generated block script.
%
%   INFO = GPR_PARSE_SCRIPT(SCRIPT) parses the first "function" line of a
%   MATLAB Function block script and returns
%       .fname   primary function name (always 'fcn' in this package)
%       .outputs cell array of output port names
%       .inputs  cell array of input port names
%       .nout    number of output ports
%       .nin     number of input ports
%       .locals  cell array of local (sub) function names in the script
%
%   build_realistic_model.m uses this to verify that the number of wires it
%   draws matches the number of ports the script declares, so a wiring
%   mistake fails with a clear message instead of a broken model.

if ~ischar(script)
    script = char(script);
end

lines = strsplit(script, '\n');
first = 0;
for k = 1:numel(lines)
    t = strtrim(lines{k});
    if strncmpi(t, 'function', 8)
        first = k;
        break;
    end
end
if first == 0
    error('gpr_parse_script:noFunction', 'Script does not contain a function line.');
end
sig = strtrim(lines{first});
% Drop a trailing comment on the signature line, if any.
cmt = strfind(sig, '%');
if ~isempty(cmt), sig = strtrim(sig(1:cmt(1)-1)); end

info = struct();
info.fname = '';
info.outputs = {};
info.inputs = {};

eq = strfind(sig, '=');
op = strfind(sig, '(');
cp = strfind(sig, ')');
if isempty(op)
    error('gpr_parse_script:badSignature', 'Cannot parse signature: %s', sig);
end
if ~isempty(eq) && eq(1) < op(1)
    % function [a, b] = fcn(x, y)
    info.outputs = split_args(sig(9:eq(1)-1));          % between "function" and "="
    info.fname   = strtrim(sig(eq(1)+1:op(1)-1));       % between "=" and "("
else
    % function fcn(x, y)
    info.fname   = strtrim(sig(9:op(1)-1));
end
if ~isempty(cp) && cp(end) > op(1)
    info.inputs = split_args(sig(op(1)+1:cp(end)-1));
end
info.nout = numel(info.outputs);
info.nin  = numel(info.inputs);
if info.nout == 0
    error('gpr_parse_script:noOutputs', 'Block script %s has no outputs.', info.fname);
end

% Local (sub) functions, e.g. the Bessel helper used by code_rangeproc.
info.locals = {};
for k = first+1:numel(lines)
    t = strtrim(lines{k});
    if strncmpi(t, 'function', 8)
        tok = regexp(t, 'function\s+([A-Za-z]\w*)\s*\(', 'tokens', 'once');
        if ~isempty(tok), info.locals{end+1} = tok{1}; end %#ok<AGROW>
        tok2 = regexp(t, 'function\s+\[[^\]]*\]\s*=\s*([A-Za-z]\w*)', 'tokens', 'once');
        if ~isempty(tok2), info.locals{end+1} = tok2{1}; end %#ok<AGROW>
        tok3 = regexp(t, 'function\s+[A-Za-z]\w*\s*=\s*([A-Za-z]\w*)', 'tokens', 'once');
        if ~isempty(tok3), info.locals{end+1} = tok3{1}; end %#ok<AGROW>
    end
end
end

function args = split_args(s)
s = strrep(strrep(s, '[', ''), ']', '');
args = {};
if isempty(strtrim(s)), return; end
parts = strsplit(s, ',');
for k = 1:numel(parts)
    t = strtrim(parts{k});
    if ~isempty(t), args{end+1} = t; end %#ok<AGROW>
end
end
