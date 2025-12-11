ECE 504 Final Project – Helicopter Control System

This repository contains the MATLAB implementation and report for the ECE 504 Final Project at Worcester Polytechnic Institute. The project focuses on analyzing and designing controllers for a linearized helicopter model using state-space techniques, optimal control, and observer-based output feedback.

Project Overview

The goal of this project is to study the behavior of a multivariable helicopter system and design a full-state feedback control architecture capable of stabilizing the system and tracking reference commands. The helicopter model is an 8-state, 4-input, 6-output linearized representation of a twin-engine rotorcraft.

The project consists of the following main components:

Loading and examining the helicopter state-space model

Open-loop stability analysis using eigenvalues and BIBO tests

Transfer function extraction for individual input–output channels

Open-loop step response analysis

Controllability and observability verification

LQR state-feedback controller design

Full-state estimator (Luenberger observer) design

Dynamic output-feedback controller construction

Closed-loop simulations for step responses and disturbance rejection

Reference tracking for the first output channel using feedforward gain design

Repository Structure
.
├── src/
│   ├── helecopterModel.m             # Helicopter state-space matrices
│   ├── final_project.m               # Main MATLAB script running all parts
│   ├── figs/                         # Auto-generated PNG figures from MATLAB
│   └── utils/                        # Optional helper functions
│
├── report/
│   ├── ECE504_FinalProject_Report.tex   # Full LaTeX project report
│   └── ECE504_FinalProject_Report.pdf   # Compiled report (if included)
│
├── README.md
└── LICENSE (optional)

MATLAB Script Description

The main script (final_project.m) completes all portions of the assignment:

Part 1: Load State-Space Model

Imports A, B, C, D matrices using helecopterModel().

Part 2: Stability Analysis

Computes eigenvalues of A and evaluates asymptotic and BIBO stability.

Part 3: Transfer Function

Extracts the first input–output transfer function 
𝐺11(𝑠)
G
11
	​

(s).

Part 4: Open-Loop Response

Simulates step responses for all outputs under a combined step input.

Part 5: Controllability/Observability

Checks rank conditions for system controllability and observability.

Part 6: LQR Controller

Designs a stabilizing LQR gain matrix K, verifies closed-loop eigenvalues, and simulates step responses.

Part 7: State Estimator

Constructs a full-order Luenberger observer with appropriately placed poles.

Part 8: Output Feedback Control

Builds the dynamic compensator (observer + state feedback) and simulates closed-loop performance.

Part 9: Zero-Input Response

Simulates disturbance rejection using initial conditions.

Part 10: Reference Tracking

Computes and applies a feedforward gain N to ensure 
𝑦1(𝑡)
y
1
	​

(t) tracks step commands with zero steady-state error.

All figures generated are automatically saved as high-resolution PNG files.

How to Run the Code

Clone the repository:

git clone https://github.com/yourusername/ECE504-HelicopterControl.git


Open MATLAB and navigate to the src/ directory.

Run the main script:

final_project


All plots will appear automatically and be saved in the figs/ folder.

Requirements

MATLAB R2020a or newer

Control System Toolbox

Symbolic Math Toolbox (optional, minimal use)

Results Summary

Key outcomes of the project:

The helicopter system is open-loop unstable.

The system is fully controllable and observable.

LQR feedback stabilizes the system and produces well-damped dynamics.

The full-state estimator reconstructs all states accurately.

The combined observer-based controller achieves stable output feedback control.

The feedforward gain ensures exact reference tracking for the first output channel.

All results are documented in the final report with accompanying figures.
