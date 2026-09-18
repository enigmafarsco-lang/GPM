function w = gpr_range_window(name, n, beta)
%GPR_RANGE_WINDOW  Spectral taper used for range compression (block B10).
%
%   w = gpr_range_window(NAME, N, BETA) returns the length-N row vector
%   applied to the calibrated SFCW spectrum before the IFFT.
%
%   Why a taper is needed
%   ---------------------
%   Range compression is an IFFT over the swept tones.  An untapered
%   (rectangular) spectrum has -13 dB range sidelobes that decay only as
%   1/delta.  A strong shallow target therefore leaves a sidelobe skirt that
%   the Kirchhoff migration sums coherently over its whole aperture, raising
%   the local noise estimate around weak deep targets until CFAR can no
%   longer see them.  Tapering trades mainlobe broadening for 20-70 dB of
%   sidelobe suppression.
%
%   Supported NAMEs: 'rectangular'/'none', 'hann', 'hamming', 'blackman',
%   'bartlett', 'kaiser' (uses BETA; default 6).
%
%   This single definition is shared by the generated Simulink block, the
%   reference runner and gpr_check_config (which measures the resulting
%   resolution numerically), so the reported resolution always matches the
%   processing that was actually applied.

  if nargin < 3 || isempty(beta), beta = 6; end
  name = lower(char(name));
  if strcmp(name,'none'), name = 'rectangular'; end
  if n < 2
    w = ones(1, max(n,1));
    return;
  end
  k = (0:n-1);
  switch name
    case 'rectangular'
      w = ones(1, n);
    case 'hann'
      w = 0.5 * (1 - cos(2*pi*k/(n-1)));
    case 'hamming'
      w = 0.54 - 0.46 * cos(2*pi*k/(n-1));
    case 'blackman'
      w = 0.42 - 0.5*cos(2*pi*k/(n-1)) + 0.08*cos(4*pi*k/(n-1));
    case 'bartlett'
      w = 1 - abs(2*k/(n-1) - 1);
    case 'kaiser'
      m = (n-1)/2;
      arg = beta * sqrt(max(0, 1 - ((k - m)/m).^2));
      w = bessel_i0(arg);
      w = w / max(w);
    otherwise
      error('GPR:RangeWindow', ...
        'Unsupported range_window ''%s'' (rectangular, hann, hamming, blackman, bartlett, kaiser).', name);
  end
  w = w(:).';
end

function y = bessel_i0(x)
%BESSEL_I0  Modified Bessel function of the first kind, order zero (series).
%   Inlined so that the generated MATLAB Function block needs no toolbox.
  y = ones(size(x));
  t = ones(size(x));
  for k = 1:40
    t = t .* (x/2).^2 / k^2;
    y = y + t;
  end
end
