function src = code_rangeproc(cfg)
%CODE_RANGEPROC  Generator for the Range Processing block (B10).
%
%   Inputs : u    - calibrated complex spectrum, n_tones x n_positions (B09)
%            beta - Kaiser beta from the P_KAISER constant port (used only
%                   when cfg.range_window is 'kaiser')
%   Output : rp   - range profiles, n_ifft x n_positions complex
%
%   Range compression is a zero-padded IFFT along the frequency axis of the
%   tapered spectrum.  The taper is cfg.range_window (gpr_range_window): an
%   untapered spectrum has -13 dB range sidelobes whose skirt the migration
%   sums coherently over its aperture, inflating the CFAR noise estimate
%   around weak deep targets until they disappear.
%   The amplitude is normalised by sum(w) so a tone of amplitude A produces a
%   peak of A whatever the window, keeping B09's calibration meaningful.

  wname = lower(gpr_cfg_value(cfg, 'range_window', 'kaiser'));
  if strcmp(wname,'none'), wname = 'rectangular'; end
  names = {'rectangular','hann','hamming','blackman','bartlett','kaiser'};
  if ~any(strcmp(wname, names))
    error('GPR:RangeWindow', ...
      'cfg.range_window ''%s'' is not supported (use one of: %s).', ...
      gpr_cfg_value(cfg,'range_window','kaiser'), strjoin(names, ', '));
  end

  % The window is baked into the script as a numeric literal: a persistent
  % cache is forbidden inside a MATLAB Function block (MATLAB Coder rejects
  % any read of a persistent variable that is not an isempty guard), and a
  % run-time call would need the generator on the coder path.  The beta input
  % port stays for wiring compatibility; the value it carries equals
  % cfg.kaiser_beta, which is what the literal was computed with.
  w = gpr_range_window(wname, cfg.n_tones, cfg.kaiser_beta);
  wgain = cfg.n_ifft/sum(w);
  wlit = strtrim(sprintf('%.17g ', w));
  src = sprintf([...
    'function rp = fcn(u, beta)\n' ...
    '%%#codegen\n' ...
    '%% Range compression: windowed, zero-padded IFFT along the frequency axis.\n' ...
    '%% Window ''%s'' with beta = %.6g, baked in at build time (no run-time\n' ...
    '%% cache, no toolbox call).  beta port kept for wiring parity.\n' ...
    'w = [%s].'';\n' ...
    'x = complex(real(u), imag(u)) .* w;\n' ...
    'rp = ifft(x, %d, 1) * %.17g;\n' ...
    'end\n'], wname, cfg.kaiser_beta, wlit, cfg.n_ifft, wgain);
end
