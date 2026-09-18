function [eps_r, eps_i, alpha, beta, v] = soil_permittivity(f, moisture, sigma)
% SOIL_PERMITTIVITY  Frequency-dependent soil model shared by the whole package.
%
%   [EPS_R, EPS_I, ALPHA, BETA, V] = SOIL_PERMITTIVITY(F, MOISTURE, SIGMA)
%   returns, at frequency F (Hz), volumetric water content MOISTURE (m^3/m^3)
%   and bulk conductivity SIGMA (S/m):
%
%       EPS_R  real relative permittivity (Dobson-type mixing model)
%       EPS_I  imaginary part from bulk conductivity (loss tangent = EPS_I/EPS_R)
%       ALPHA  field attenuation constant, Np/m
%       BETA   phase constant, rad/m
%       V      phase velocity, m/s
%
%   The exact same formulas are baked into the generated Simulink blocks
%   (code_channel.m, code_migration.m, code_detection.m, code_report.m) so
%   that the feasibility check, the Simulink model and the reference
%   simulation all agree. If you change something here, change it there too.
%
%   Validity: low-loss dielectric approximation (tan(delta) < ~0.5), which
%   covers dry to moderately wet sand/loam in the 1 MHz - 3 GHz GPR bands.

c0   = 299792458;
mu0  = 4*pi*1e-7;
eps0 = 8.854187817e-12;

% Dobson mixing model: eps_r = (1 + (rho_b/rho_s)*(eps_s^a - 1)
%                                + theta^b' * eps_fw^a - theta)^(1/a)
% with a = 0.65, b' = 1.27 (sandy soil), eps_s = 3 (dry soil), rho_b/rho_s = 1.5/2.65.
a_exp   = 0.65;
b_exp   = 1.27;
tau_w   = 0.6e-10;                       % bound-water relaxation time
omega   = 2*pi*f(:).';
eps_fw  = 80 ./ (1 + (omega*tau_w).^2);   % real part of free-water permittivity

eps_r = (1 + 1.5/2.65*(3^a_exp - 1) + moisture.^b_exp .* eps_fw.^a_exp - moisture).^(1/a_exp);
eps_r = real(eps_r);
eps_r(eps_r < 3)  = 3;                    % dry-soil floor
eps_r(eps_r > 30) = 30;                   % saturated/clay ceiling

% The imaginary part is the conduction loss plus a numerical floor.  The
% Debye relaxation loss of the water content is deliberately NOT added: at
% the low moisture values of the shipped configurations (0.05-0.08 m^3/m^3)
% it would over-state attenuation by an order of magnitude unless the
% free/bound-water split of Dobson is modelled in full.  The consequence is
% conservative for dry soils: attenuation grows with frequency and with
% conductivity, but only weakly with moisture.
eps_i = sigma ./ (omega*eps0);
eps_i(eps_i < 0.01) = 0.01;               % loss floor (numerical + realistic)

lt = eps_i ./ eps_r;                       % loss tangent
common = omega .* sqrt(mu0*eps0*eps_r/2);
alpha = common .* sqrt(sqrt(1 + lt.^2) - 1);   % Np/m
beta  = common .* sqrt(sqrt(1 + lt.^2) + 1);   % rad/m
v     = c0 ./ sqrt(eps_r);                     % m/s
end
