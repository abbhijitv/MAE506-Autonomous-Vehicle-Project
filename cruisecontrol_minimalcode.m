%% Cruise Control 

clc; clear; close all


% 1. Define system parameters
m = 1500;       % Vehicle mass (kg)
b = 50;         % Linear drag coefficient
c = 0.4;        % Quadratic drag coefficient
tau = 0.5;      % Actuator time constant (s)
v_eq = 25;      % Equilibrium velocity (m/s)

fprintf("Vehicle mass is: %.1f kg \n",m)
fprintf("Linear Resistance is: %.1f \n",b)
fprintf("Quadratic Resisitance is: %.1f \n",c)
fprintf("Actuator time  is: %.1f \n",tau)
fprintf("Equilibrium velocity is: %f m/s\n", v_eq);
dragForce = b*v_eq + c*v_eq^2;
fprintf("Equilibrium drag force is: %f N\n", dragForce);

% Define state-space matrices
drag_slope = b + 2*c*abs(v_eq);

A = [-drag_slope/m,  1/m;
      0,            -1/tau];

B = [0;
     1/tau];

C = [1 0];

D = 0;

% Display matrices
disp('A Matrix:'); disp(A);
disp('B Matrix:'); disp(B);
disp('C Matrix:'); disp(C);
disp('D Matrix:'); disp(D);

% Controllability
Co = ctrb(A,B);
rank_Co = rank(Co);

disp('Controllability Matrix:'); disp(Co);
disp('Controllability Rank:'); disp(rank_Co);

if rank_Co == size(A,1)
    disp('System is CONTROLLABLE');
else
    disp('System is NOT CONTROLLABLE');
end

% Observability
Ob = obsv(A,C);
rank_Ob = rank(Ob);

disp('Observability Matrix:'); disp(Ob);
disp('Observability Rank:'); disp(rank_Ob);

if rank_Ob == size(A,1)
    disp('System is OBSERVABLE');
else
    disp('System is NOT OBSERVABLE');
end

% Eigenvalues and stability
eigenvalues = eig(A);

disp('Eigenvalues:'); disp(eigenvalues);

if all(real(eigenvalues) < 0)
    disp('System is ASYMPTOTICALLY STABLE');
elseif any(real(eigenvalues) > 0)
    disp('System is UNSTABLE');
else
    disp('System is MARGINALLY STABLE or INCONCLUSIVE');
end

% Display TF

tf(ss(A,B,C,D))

% Phase Plot
time = linspace(0, 100, 1000);
initialConditions = [
     5, -5,  0,  0,  5, -5;
     0,  0,  2, -2,  2, -2
];

figure;
hold on
grid on
for i = 1:6
    x0 = initialConditions(:,i);
    for j = 1:length(time)
        test(:, j) = expm(A * time(j)) * x0;
    end

    plot(test(1,:),test(2,:))
end

title('System')
legend('x1','x2','x3','x4','x5','x6')

% Creating Controller

