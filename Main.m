%% =======================================================================
%   ECE 505 Final Project – Control System Toolbox Version (Modularized)
%   Dhruv Madan
%   Full Analysis + LQR + Estimator + Output Feedback + Tracking
% ========================================================================

clear; clc; close all;

%% -----------------------------------------------------------------------
%   Part 1 — Load helicopter system
% ------------------------------------------------------------------------
[A, B, C, D, sys] = loadHelicopterSystem();
disp("State-space system loaded.");
disp(sys);
newline();


%% -----------------------------------------------------------------------
%   Part 2 & 3 — Stability analysis (Eigenvalues + BIBO)
% ------------------------------------------------------------------------
[eigVals, asymStable, biboStable] = checkStability(A, sys);
newline();


%% -----------------------------------------------------------------------
%   Part 4 — Transfer function G11(s)
% ------------------------------------------------------------------------
disp("Part 4:");

G = tf(sys);     % full TF matrix
G11 = G(1,1);    % extract G11

G11
newline();

%% -----------------------------------------------------------------------
%   Part 5 — Step response (all inputs stepped)
% ------------------------------------------------------------------------
plotStepAllInputs(A, B, C, D, 10);
newline();


%% -----------------------------------------------------------------------
%   Part 6 — Controllability & Observability
% ------------------------------------------------------------------------
[ctrlOK, obsOK] = checkControllabilityObservability(A, B, C);
newline();


%% -----------------------------------------------------------------------
%   Part 7 — LQR state-feedback design
% ------------------------------------------------------------------------
[K, e_cl] = designLQR(A, B, C);
newline();


%% -----------------------------------------------------------------------
%   Part 8 — Design estimator (state observer)
% ------------------------------------------------------------------------
L = designEstimator(A, C, e_cl);
newline();


%% -----------------------------------------------------------------------
%   Part 9 — Output-feedback closed-loop step response
% ------------------------------------------------------------------------
[A_aug, B_aug, C_aug, D_aug] = buildAugmentedOutputFeedbackSystem(A, B, C, D, K, L);
plotClosedLoopStep(A_aug, B_aug, C_aug, D_aug, 10);
newline();


%% -----------------------------------------------------------------------
%   Part 10 — Zero-input response (initial conditions)
% ------------------------------------------------------------------------
x0 = [1.2; 2.5; 3.1; -0.5; -1; 4.9; -2; -0.7];
simulateZeroInput(A_aug, C_aug, x0, 10);
newline();


%% -----------------------------------------------------------------------
%   Part 11 — Reference tracking controller (unity feedback)
% ------------------------------------------------------------------------
simulateReferenceTracking(A, B, C, D, K, L, 10);
newline();