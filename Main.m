%% =======================================================================
%   ECE 505 Final Project – Control System Toolbox Version
%   Full Analysis + LQR + Estimator + Output Feedback + Tracking
% ========================================================================

clear; clc;
syms s;

%% =======================
%  Global Plot Settings
% =======================
set(0, 'DefaultLineLineWidth', 2.5);
set(0, 'DefaultAxesFontSize', 18);
set(0, 'DefaultTextFontSize', 20);
set(0, 'DefaultAxesLabelFontSizeMultiplier', 1.3);
set(0, 'DefaultAxesTitleFontSizeMultiplier', 1.4);
set(0, 'DefaultFigureColor', 'w'); % white background


%% =======================================================================
%   Part 1 – Load State-Space Model
% ========================================================================

[A, B, C, D] = helecopterModel();
sys = ss(A, B, C, D);

disp("State-space system loaded.");
sys
fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 2 – Stability via Eigenvalues of A
% ========================================================================

disp("Part 2:");
lambda = eig(A);

disp("Eigenvalues of A:");
disp(lambda);

if all(real(lambda) < 0)
    disp("The system is asymptotically stable.");
else
    disp("The system is NOT asymptotically stable.");
end

fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 3 – BIBO Stability Check
% ========================================================================

disp("Part 3:");

if isstable(sys)
    disp("System is BIBO Stable.");
else
    disp("System is BIBO UNSTABLE.");
end

fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 4 – Compute Transfer Function G11(s)
% ========================================================================

disp("Part 4:");

G = tf(sys);     % full TF matrix
G11 = G(1,1);    % extract G11

G11
fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 5 – Step Response (All Inputs = Step)
% ========================================================================

disp("Part 5: Step Response");
fprintf('\n----------------------------------\n');

u = ones(size(B,2),1);      % combine all inputs
sys_sum = ss(A, B*u, C, D*u);

figure('Name','Part 5: Step Response (All Inputs)');
stepplot(sys_sum, 10);
title("Part 5: Output Response to Unit Step in All Inputs");
grid on;


%% =======================================================================
%   Part 6 – Controllability & Observability
% ========================================================================

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


%% =======================================================================
%   Part 7 – State-Feedback LQR Controller
% ========================================================================

disp("Part 7: State-Feedback Control (LQR)");
fprintf('\n----------------------------------\n');

if rank(ctrb(A,B)) ~= size(A,1)
    error('System is NOT fully controllable. Cannot design LQR.');
else
    disp("System is controllable — safe for LQR.");
end

Q = C' * C;                     % output weighting
R = 0.1 * eye(size(B,2));       % input penalty

[K, S, e_cl] = lqr(A, B, Q, R);

disp("LQR Gain Matrix K:");
disp(K);

disp("Closed-loop eigenvalues of (A - B*K):");
disp(e_cl);

Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);

if all(real(e_cl) < 0)
    disp("Closed-loop system is asymptotically stable.");
else
    warning("Closed-loop system NOT stable — adjust Q/R.");
end

figure('Name','Part 7: Closed-Loop Step Response (LQR)');
step(sys_cl(:,1));
title("Part 7: Closed-Loop Step Response with LQR Control");
grid on;

fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 8 – Full-State Estimator (Observer) + Output Feedback
% ========================================================================

disp("Part 8: Estimator and Output-Feedback Controller");
fprintf('\n----------------------------------\n');

if rank(obsv(A,C)) ~= size(A,1)
    error("System is NOT observable — cannot design estimator.");
else
    disp("System is observable — estimator OK.");
end

desired_estimator_poles = 3 * e_cl;
L = place(A', C', desired_estimator_poles).';

disp("Observer Gain L:");
disp(L);

% Dynamic controller (y → u)
A_est = A - B*K - L*C;
B_est = L;
C_est = -K;
D_est = zeros(size(K,1), size(C,1));
controller = ss(A_est, B_est, C_est, D_est);

disp("Dynamic output-feedback controller created.");

A_aug = [A      -B*K;
         L*C  A - B*K - L*C];

eig_aug = eig(A_aug);
disp("Eigenvalues of combined plant+observer:");
disp(eig_aug);

if all(real(eig_aug) < 0)
    disp("Output-feedback closed-loop system is STABLE.");
else
    warning("Output-feedback system NOT stable — re-tune.");
end

B_aug = [B; B];
C_aug = [C, zeros(size(C))];
D_aug = D;

sys_output_feedback = ss(A_aug, B_aug, C_aug, D_aug);

figure('Name','Part 8: Output Feedback Response');
step(sys_output_feedback(:,1), 10);
title("Part 8: Closed-Loop Response with Output Feedback");
grid on;

fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 9 – Output Feedback Closed-Loop Step Response
% ========================================================================

disp("Part 9: Closed-loop Step Response (Estimator + State Feedback)");
fprintf('\n----------------------------------\n');

A_aug = [A - B*K,     -B*K;
         L*C,         A - B*K - L*C];

B_aug = [B;
         zeros(size(B))];

C_aug = [C, zeros(size(C))];
D_aug = D;

sys_ofb = ss(A_aug, B_aug, C_aug, D_aug);

figure('Name','Part 9: Output Feedback Step Response');
step(sys_ofb(:,1), 10);
title("Part 9: Closed-Loop Output Response with Output Feedback");
grid on;

fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 10 – Zero-Input Response (Initial Condition Response)
% ========================================================================

disp("Part 10: Zero-Input Response (Output-Feedback Closed Loop)");
fprintf('\n----------------------------------\n');

A_aug = [A - B*K,     -B*K;
         L*C,         A - B*K - L*C];

B_zero = zeros(size(A_aug,1), size(B,2));
C_aug = [C, zeros(size(C))];
D_zero = zeros(size(C,1), size(B,2));

sys_zero_input = ss(A_aug, B_zero, C_aug, D_zero);

x0 = [1.2; 2.5; 3.1; -0.5; -1; 4.9; -2; -0.7];
xhat0 = zeros(size(A,1),1);
X0_aug = [x0; xhat0];

figure('Name','Part 10: Zero-Input Response');
initial(sys_zero_input, X0_aug, 10);
title("Part 10: Zero-Input Response of Closed-Loop System");
grid on;

fprintf('\n----------------------------------\n');


%% =======================================================================
%   Part 11 – Reference Tracking for y1(t)
% ========================================================================

disp("Part 11: Reference Tracking Controller for y1(t)");
fprintf('\n----------------------------------\n');

Acl = A - B*K;
b1 = B(:,1);
c1 = C(1,:);

N = 1 / (c1 * (-Acl \ b1));
disp("Reference Gain N:");
disp(N);

A_aug = [A - B*K,     -B*K;
         L*C,         A - B*K - L*C];

B_ref = [b1*N;
         zeros(size(b1))];

C_y1 = [c1, zeros(1, size(A,1))];
D_y1 = 0;

sys_track = ss(A_aug, B_ref, C_y1, D_y1);

figure('Name','Part 11: Reference Tracking (y1)');
step(sys_track, 10);
title("Part 11: Tracking Response — y_1(t) Following a Unit Step");
xlabel("Time (s)");
ylabel("y_1(t)");
grid on;

fprintf('\n----------------------------------\n');