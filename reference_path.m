function paths = reference_path(cfg)
% generate both roads with analytic reference headings
validateattributes(cfg.path_length, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(cfg.path_amplitude, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(cfg.path_wavelength, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});

% use the configured time grid to set the number of path samples
X = linspace(0, cfg.path_length, numel(cfg.t));
paths.straight.X = X;
paths.straight.Y = zeros(size(X));
paths.straight.psi = zeros(size(X));

k = 2*pi/cfg.path_wavelength;
paths.curved.X = X;
paths.curved.Y = cfg.path_amplitude*sin(k*X);
paths.curved.psi = atan(cfg.path_amplitude*k*cos(k*X));
end
