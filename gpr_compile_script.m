function [fh, info] = gpr_compile_script(script, out_name, workdir)
% GPR_COMPILE_SCRIPT  Make a generated block script callable from MATLAB.
%
%   [FH, INFO] = GPR_COMPILE_SCRIPT(SCRIPT, OUT_NAME, WORKDIR) writes SCRIPT
%   to WORKDIR/OUT_NAME.m with its primary function renamed to OUT_NAME, puts
%   WORKDIR on the path and returns a function handle to it. INFO is the
%   signature description returned by gpr_parse_script.
%
%   This is what lets simulate_pipeline_reference.m execute exactly the same
%   code text that build_realistic_model.m pastes into the Simulink MATLAB
%   Function blocks - one implementation, two execution environments.

if nargin < 3 || isempty(workdir)
    workdir = fullfile(tempdir, 'gpr_pipeline_ref');
end
if ~exist(workdir, 'dir')
    mkdir(workdir); %#ok<CTCH>
end

info = gpr_parse_script(script);

lines = strsplit(script, '\n');
done = false;
for k = 1:numel(lines)
    t = strtrim(lines{k});
    if strncmpi(t, 'function', 8)
        esc = regexptranslate('escape', info.fname);
        before = lines{k};
        % "function [a, b] = fcn(...)": rename the function, not the outputs.
        lines{k} = regexprep(before, ['(function[^\n=]*=\s*)' esc], ['$1' out_name], 'once');
        if strcmp(lines{k}, before)
            % "function fcn(...)" (no outputs).
            lines{k} = regexprep(before, ['(function\s*)' esc], ['$1' out_name], 'once');
        end
        if strcmp(lines{k}, before)
            error('gpr_compile_script:rename', ...
                'Could not rename ''%s'' in: %s', info.fname, before);
        end
        done = true;
        break;
    end
end
if ~done
    error('gpr_compile_script:noFunction', 'Script has no function line.');
end
new_script = strjoin(lines, '\n');

file = fullfile(workdir, [out_name '.m']);
fid = fopen(file, 'w');
if fid < 0
    error('gpr_compile_script:cannotWrite', 'Cannot write %s', file);
end
fwrite(fid, new_script);
fclose(fid);

if isempty(strfind(path, workdir)) %#ok<STREMP>
    addpath(workdir);
end
try
    rehash;
catch
end

fh = str2func(out_name);
end
