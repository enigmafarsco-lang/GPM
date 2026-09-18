function [broadening, null_bins, guard_bins] = gpr_pulse_width(name, n, beta, n_ifft)
%GPR_PULSE_WIDTH  Compressed-pulse width of a range-compression window.
%
%   [BROADENING, NULL_BINS, GUARD_BINS] = gpr_pulse_width(NAME, N, BETA, N_IFFT)
%   measures the window returned by gpr_range_window and reports
%     BROADENING : -3 dB width of the tapered pulse divided by the -3 dB
%                  width of the rectangular pulse (the range-resolution
%                  penalty of the taper; 1.0 for 'rectangular').
%     NULL_BINS  : null-to-null mainlobe width in bins of the N-point DFT
%                  (multiply by n_ifft/n_tones to get IFFT bins).
%     GUARD_BINS : minimum CFAR guard band in IFFT bins - the -3 dB
%                  half-width of the compressed pulse, rounded up.  Cells
%                  inside the guard are excluded from the noise estimate, so
%                  a target cannot mask itself.  Requires N_IFFT.
%
%   Everything is measured from a heavily zero-padded FFT of the window, so
%   any window/beta combination is handled exactly and the configuration
%   report can never disagree with the processing actually performed.

  if nargin < 3 || isempty(beta), beta = 6; end
  if nargin < 4, n_ifft = n; end
  nf = 2^nextpow2(max(64*n, 4096));

  w  = gpr_range_window(name, n, beta);
  wr = gpr_range_window('rectangular', n, beta);

  f3  = width_at(fft(w,  nf), nf, n, 1/sqrt(2));
  f3r = width_at(fft(wr, nf), nf, n, 1/sqrt(2));
  broadening = f3/f3r;

  nullw  = width_at(fft(w, nf), nf, n, 0, 'null');
  null_bins = nullw;

  % -3 dB half-width expressed in IFFT bins of length n_ifft
  guard_bins = ceil(0.5*f3*n_ifft/n);
end

function wdt = width_at(X, nf, n, level, mode)
  if nargin < 5, mode = 'level'; end
  F = abs(fftshift(X));
  F = F/max(F);
  [~, ip] = max(F);
  if strcmp(mode,'null')
    j = ip; while j < numel(F) && F(j) >= F(j+1), j = j + 1; end   %#ok<WNTAG>
    l = ip; while l > 1     && F(l) >= F(l-1), l = l - 1; end       %#ok<WNTAG>
  else
    j = ip; while j < numel(F) && F(j) > level, j = j + 1; end      %#ok<WNTAG>
    l = ip; while l > 1     && F(l) > level, l = l - 1; end         %#ok<WNTAG>
  end
  wdt = (j - l)*n/nf;
end
