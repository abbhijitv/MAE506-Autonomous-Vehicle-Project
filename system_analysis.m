function results = system_analysis(model, cfg)
% analyze only the two-state longitudinal deviation model
A = model.A;
B = model.B;
C = model.C;
D = model.D;
n = size(A, 1);
assert(isequal(size(A), [2, 2]) && isequal(size(B), [2, 1]) ...
    && isequal(size(C), [1, 2]) && isscalar(D), ...
    'Analysis requires the two-state SISO cruise model.');
assert(all(isfinite([A(:); B(:); C(:); D; model.E(:)])), ...
    'Model matrices must be finite.');

fprintf('\nSystem Analysis\n');

% stability, controllability, and observability
results.poles = eig(A);
results.stable = all(real(results.poles) < 0);
fprintf('Eigenvalues (1/s):');
for j = 1:n
    if isreal(results.poles(j))
        fprintf(' %.7g', results.poles(j));
    else
        fprintf(' %.7g%+.7gi', real(results.poles(j)), imag(results.poles(j)));
    end
end
fprintf('\n');
if results.stable
    fprintf('Open-loop system: asymptotically stable.\n');
else
    fprintf('Open-loop system: not asymptotically stable.\n');
end

results.controllability = ctrb(A, B);
results.observability = obsv(A, C);
results.controllability_rank = rank(results.controllability);
results.observability_rank = rank(results.observability);
fprintf('Controllability rank: %d/%d - ', results.controllability_rank, n);
if results.controllability_rank == n
    fprintf('Fully controllable.\n');
else
    fprintf('Not fully controllable.\n');
end
fprintf('Observability rank: %d/%d - ', results.observability_rank, n);
if results.observability_rank == n
    fprintf('Fully observable.\n');
else
    fprintf('Not fully observable.\n');
end

% infinite-horizon Gramians require a Hurwitz A matrix
results.Wc = [];
results.Wo = [];
if results.stable
    results.Wc = lyap(A, B*B');
    results.Wo = lyap(A', C'*C);
    residual_c = A*results.Wc + results.Wc*A' + B*B';
    residual_o = A'*results.Wo + results.Wo*A + C'*C;
    results.gramian_residuals = [norm(residual_c, 'fro')/max(1, norm(B*B', 'fro')), ...
        norm(residual_o, 'fro')/max(1, norm(C'*C, 'fro'))];
    assert(all(isfinite([results.Wc(:); results.Wo(:)])), 'Gramians must be finite.');
    assert(all(results.gramian_residuals < 1e-9), 'Gramian Lyapunov residual is too large.');
    results.Wc_eigenvalues = eig(results.Wc);
    results.Wo_eigenvalues = eig(results.Wo);
    if results.controllability_rank == n
        assert(all(results.Wc_eigenvalues > 0), 'Controllability Gramian must be positive definite.');
    end
    if results.observability_rank == n
        assert(all(results.Wo_eigenvalues > 0), 'Observability Gramian must be positive definite.');
    end
    results.Wc_min_eigenvalue = min(results.Wc_eigenvalues);
    results.Wo_min_eigenvalue = min(results.Wo_eigenvalues);
    results.gramians_positive_definite = results.Wc_min_eigenvalue > 0 ...
        && results.Wo_min_eigenvalue > 0;
    fprintf('\nMinimum controllability Gramian eigenvalue: %.6e\n', results.Wc_min_eigenvalue);
    fprintf('Minimum observability Gramian eigenvalue: %.6e\n', results.Wo_min_eigenvalue);
    if results.gramians_positive_definite
        fprintf('Both Gramians positive definite: yes.\n');
    else
        fprintf('Both Gramians positive definite: no.\n');
    end
else
    fprintf('\nInfinite-horizon Gramians omitted because A is not Hurwitz.\n');
end

% companion forms use delta_x = T*z
assert(results.controllability_rank == n && results.observability_rank == n, ...
    'Canonical transformations require a controllable and observable realization.');
a = poly(A);
Ac = [0, 1; -a(3), -a(2)];
Bc = [0; 1];
Tc = results.controllability/ctrb(Ac, Bc);
Cc = C*Tc;

% the dual companion form has output [0, 1]
Ao = Ac';
Co = Bc';
To = results.observability\obsv(Ao, Co);
Bo = To\B;
forms = {struct('A', Ac, 'B', Bc, 'C', Cc, 'D', D, 'T', Tc), ...
         struct('A', Ao, 'B', Bo, 'C', Co, 'D', D, 'T', To)};

% check similarity equations and rational transfer-function equality
[num_original, den_original] = tfdata(model.tf, 'v');
for j = 1:2
    form = forms{j};
    T = form.T;
    assert(all(isfinite(T(:))) && rcond(T) > eps, 'Canonical transformation is singular.');
    form.similarity_error = max([norm((T\A)*T - form.A, 'fro')/max(1, norm(form.A, 'fro')), ...
        norm(T\B - form.B)/max(1, norm(form.B)), ...
        norm(C*T - form.C)/max(1, norm(form.C))]);
    form.ss = ss(form.A, form.B, form.C, form.D);
    form.tf = tf(form.ss);
    [num_form, den_form] = tfdata(form.tf, 'v');
    left = conv(num_original, den_form);
    right = conv(num_form, den_original);
    count = max(numel(left), numel(right));
    left = [zeros(1, count - numel(left)), left];
    right = [zeros(1, count - numel(right)), right];
    form.tf_error = norm(left - right, inf)/max([norm(left, inf), norm(right, inf), eps]);
    assert(isfinite(form.similarity_error) && form.similarity_error < 1e-9, ...
        'Canonical similarity equations do not agree.');
    assert(isfinite(form.tf_error) && form.tf_error < 1e-8, ...
        'Canonical transfer function does not match the original.');
    forms{j} = form;
end
results.controllable_form = forms{1};
results.observable_form = forms{2};
fprintf('\nCanonical forms: numerically verified\n');
fprintf('Maximum similarity error: %.3e; transfer-function error: %.3e\n', ...
    max(forms{1}.similarity_error, forms{2}.similarity_error), ...
    max(forms{1}.tf_error, forms{2}.tf_error));

end
