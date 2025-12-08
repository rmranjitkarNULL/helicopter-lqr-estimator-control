%% ECE 505 Project – Control System Toolbox Version
clear; clc;
syms s

%% Part 1 – Load state-space model
[A, B, C, D] = helecopterModel();

% Build state-space system object
sys = ss(A, B, C, D);

disp("State-space system loaded.");
sys

fprintf('\n----------------------------------\n');


%% Part 2 – Stability via eigenvalues (A-matrix)
lambda = eig(A);
disp("Part 2: ")
disp("Eigenvalues of A:");
disp(lambda);

if all(real(lambda) < 0)
    disp("The system is asymptotically stable.");
else
    disp("The system is NOT asymptotically stable.");
end

fprintf('\n----------------------------------\n');


%% Part 3 – BIBO Stability using Control System Toolbox
% For state-space systems:
%   BIBO stable ⇔ all poles of sys are strictly in left-half plane
disp("Part 3:");

if isstable(sys)
    disp("System is BIBO Stable.");
else
    disp("System is BIBO UNSTABLE.");
end

fprintf('\n----------------------------------\n');


%% Part 4 – Compute transfer function G11(s)

% Convert entire system to transfer function matrix
G = tf(sys);

% Extract G11
G11 = G(1,1);

disp("Part 4:");
G11   % Toolbox prints the rational TF nicely

fprintf('\n----------------------------------\n');


%% Part 5 – Step response (all inputs = step)

disp("Part 5: Step Response");

fprintf('\n----------------------------------\n');

% If you want one combined step input for all inputs:
u = ones(size(B,2),1);

% Build a system with summed inputs:
sys_sum = ss(A, B*u, C, D*u);  

% Plot step response of all outputs
figure('Name','Part 5: Step Response (All Inputs)');
stepplot(sys_sum, 10);
title("Part 5: Output Response to Unit Step in All Inputs");
grid on;


%% Part 6 – Controllability and Observability using Toolbox

disp("Part 6:");

% Controllability
Co = ctrb(A, B);
if rank(Co) == size(A,1)
    disp("The system (A,B) is CONTROLLABLE.");
else
    disp("The system (A,B) is NOT controllable.");
end

% Observability
Ob = obsv(A, C);
if rank(Ob) == size(A,1)
    disp("The system (A,C) is OBSERVABLE.");
else
    disp("The system (A,C) is NOT observable.");
end

fprintf('\n----------------------------------\n');


%% Part 7 – Compute a stabilizing state-feedback controller (LQR)

disp("Part 7: State-Feedback Control (LQR)");
fprintf('\n----------------------------------\n');

% Check controllability (required before LQR)
Co = ctrb(A,B);
if rank(Co) ~= size(A,1)
    error('System is NOT fully controllable. Cannot guarantee stabilization.');
else
    disp("System is controllable — safe to design LQR controller.");
end

% Choose LQR weights
% Q penalizes state deviation; R penalizes control effort.
Q = C' * C;             % common heuristic: penalize outputs
R = 0.1 * eye(size(B,2));  % small penalty on inputs → more aggressive control

% Compute the LQR optimal gain matrix K
[K, S, e_cl] = lqr(A, B, Q, R);

disp("LQR Gain Matrix K:");
disp(K);

disp("Closed-loop eigenvalues of (A - B*K):");
disp(e_cl);

% Build closed-loop system with u = -Kx
Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);

% Check stability explicitly
if all(real(e_cl) < 0)
    disp("Closed-loop system is asymptotically stable with LQR.");
else
    warning("Closed-loop system is NOT stable — adjust Q and R.");
end

% Plot closed-loop step response (step on input 1)
figure('Name','Part 7: Closed-Loop Step Response (LQR)');
step(sys_cl(:,1));
title("Part 7: Closed-Loop Step Response with LQR Control");
grid on;



fprintf('\n----------------------------------\n');

%% Part 8 – State Estimator and Output-Feedback Controller

disp("Part 8: Estimator and Output-Feedback Controller");
fprintf('\n----------------------------------\n');

% Sanity check: observability
Ob = obsv(A, C);
if rank(Ob) ~= size(A,1)
    error('System is NOT fully observable. Cannot design full-order estimator.');
else
    disp("System is observable — safe to design an estimator.");
end

% Desired estimator poles:
% Make the observer faster than the state-feedback dynamics.
% Use the LQR closed-loop poles e_cl from Part 7 and move them left (3x).
desired_estimator_poles = 3 * e_cl;

% Observer gain L (dual of state feedback: use A', C')
L = place(A', C', desired_estimator_poles).';
disp("Observer gain L:");
disp(L);

% Estimator dynamics (for reference):
%   x_hat_dot = (A - B*K - L*C) x_hat + L*y
A_est = A - B*K - L*C;
B_est = L;
C_est = -K;
D_est = zeros(size(K,1), size(C,1));

% This is the dynamic controller mapping y -> u:
%   u = -K * x_hat
controller = ss(A_est, B_est, C_est, D_est);
disp("Dynamic output-feedback controller (state estimator + state feedback) created.");

% Combined closed-loop plant + estimator (for checking eigenvalues)
% States: [x; x_hat]
A_aug = [A      -B*K;
         L*C  A - B*K - L*C];

eig_aug = eig(A_aug);
disp("Eigenvalues of combined plant+observer closed-loop system:");
disp(eig_aug);

if all(real(eig_aug) < 0)
    disp("Combined output-feedback closed-loop system is asymptotically STABLE.");
else
    warning("Combined output-feedback closed-loop system is NOT stable. Re-tune poles or Q/R.");
end

% Optional: closed-loop response with output feedback
% Treat input as disturbance or reference injected at plant input
B_aug = [B; B];          % example: same input hits plant and estimator
C_aug = [C, zeros(size(C))];
D_aug = D;

sys_output_feedback = ss(A_aug, B_aug, C_aug, D_aug);

figure('Name','Part 8: Output Feedback Response');
step(sys_output_feedback(:,1), 10);
title("Part 8: Closed-Loop Response with Output Feedback (Estimator + K)");
grid on;



fprintf('\n----------------------------------\n');

%% Part 9 – Closed-loop Response with Stabilizing Output Feedback
disp("Part 9: Closed-loop Response (Estimator + State Feedback)");
fprintf('\n----------------------------------\n');

% Build the correct augmented model for output feedback
% States = [x ; x_hat]
A_aug = [A - B*K,     -B*K;
         L*C,     A - B*K - L*C];

% External input enters ONLY the plant, not the estimator
B_aug = [B;
         zeros(size(B))];

% Output matrix (we plot the PLANT output y only)
C_aug = [C, zeros(size(C))];
D_aug = D;

% Closed-loop system
sys_ofb = ss(A_aug, B_aug, C_aug, D_aug);

% Step response to a unit step on input channel 1
figure('Name','Part 9: Output Feedback Step Response');
xlabel("Time (s)");
ylabel("Plant Outputs y");
step(sys_ofb(:,1), 10);
title("Part 9: Closed-Loop Output Response with Output Feedback (u = -K x̂)");
grid on;

fprintf('\n----------------------------------\n');

%% Part 10 – Zero-Input Response with Output Feedback
disp("Part 10: Zero-Input Response (Output-Feedback Closed Loop)");
fprintf('\n----------------------------------\n');

% Augmented closed-loop system matrices (ensure consistent with Part 9)
A_aug = [A - B*K,     -B*K;
         L*C,     A - B*K - L*C];

% No external input → B_aug * u(t) = 0
B_aug_zero = zeros(size(A_aug,1), size(B,2));

% Output matrix (plant output only)
C_aug = [C, zeros(size(C))];
D_aug = zeros(size(C,1), size(B,2));

% Build zero-input system
sys_zero_input = ss(A_aug, B_aug_zero, C_aug, D_aug);

% Initial condition for plant x(0)
x0 = [1.2; 2.5; 3.1; -0.5; -1; 4.9; -2; -0.7];

% Estimator initial condition (typically zero)
xhat0 = zeros(size(A,1),1);

% Full augmented initial condition
X0_aug = [x0; xhat0];

figure('Name','Part 10: Zero-Input Response');
initial(sys_zero_input, X0_aug, 10);
title("Part 10: Zero-Input Response of Closed-Loop System (Output Feedback)");
xlabel("Time (s)");
ylabel("Plant Outputs y(t)");
grid on;


fprintf('\n----------------------------------\n');

%% Part 11 – Reference Tracking for y1(t) in Unity-Feedback Form
disp("Part 11: Reference Tracking Controller for y1(t)");
fprintf('\n----------------------------------\n');

% Closed-loop A matrix already defined earlier:
Acl = A - B*K;

% Extract input channel 1 and output channel 1
b1 = B(:,1);
c1 = C(1,:);

% Compute feedforward reference gain N (for zero steady-state tracking error)
N = 1 / (c1 * (-Acl \ b1));
disp("Reference Gain N:");
disp(N);

% Build augmented output-feedback system including estimator

A_aug = [A - B*K,     -B*K;
         L*C,     A - B*K - L*C];

% Reference enters only through input 1 → amplified by N
B_ref = [b1 * N;
         zeros(size(b1))];    % estimator does NOT get reference

% Output is plant y1(t) only
C_y1 = [c1, zeros(1, size(A,1))];
D_y1 = 0;

% Closed-loop system from reference r(t) → output y1(t)
sys_track = ss(A_aug, B_ref, C_y1, D_y1);

% Plot step response (reference = unit step)

figure('Name','Part 11: Reference Tracking (y1)');
step(sys_track, 10);
title("Part 11: Tracking Response — y_1(t) Following a Unit Step Reference");
xlabel("Time (s)");
ylabel("y_1(t)");
grid on;


fprintf('\n----------------------------------\n');