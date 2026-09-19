function v = gpr_version()
% GPR_VERSION  Version stamp of this package installation.
%
%   v = GPR_VERSION() returns the release string.  validate_package and
%   gpr_realistic_main print it, so a stale copy of the package on disk or
%   on the MATLAB path identifies itself immediately in any log.
%
%   history
%     1.0  first validated release (77 checks, 12 tests)
%     1.1  explicit Simulink chart I/O sizes, TP scopes, n_ifft rule (79)
%     1.2  MATLAB-legal map indexing, path-shadow guard (80)
%     1.3  B10 persistent-cache coder fix, window baked in (81)
%     1.4  version stamp printed by validate_package / gpr_realistic_main
%     1.5  coder type stability (B11/B12), name-based chart I/O stamping
%     1.6  offline MATLAB-Coder check in validate_package; unfinished models
%          parked in tempdir; fixes merged into main
v = '1.6';
end
