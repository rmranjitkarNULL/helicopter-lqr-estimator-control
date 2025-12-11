# ECE 504 Final Project – Helicopter Control System

This repository contains the MATLAB implementation and report for the ECE 504 Final Project at Worcester Polytechnic Institute. The project analyzes a linearized helicopter model and designs a complete state-space control architecture, including LQR regulation, full-state estimation, and output-feedback tracking control.

---

## Overview

The objective of this project is to study the dynamics of an unstable multivariable helicopter system and develop a stabilizing control strategy. The helicopter model is an 8-state, 4-input, 6-output linearized representation of a twin-engine rotorcraft. The project includes:

- Stability analysis through eigenvalue evaluation  
- BIBO stability assessment  
- Transfer function extraction  
- Open-loop system simulation  
- Controllability and observability verification  
- LQR controller design  
- Full-state Luenberger observer design  
- Dynamic output-feedback controller synthesis  
- Zero-input (disturbance) response analysis  
- Reference tracking using feedforward gain design  

The MATLAB code automatically generates plots and saves figures for use in the final report.

---

## MATLAB Script Description

The primary script is `src/Main.m`, which executes all sections of the project. A summary of each part is included below:

### Part 1: Load State-Space Model
Loads matrices \( A, B, C, D \) using `helecopterModel()`.

### Part 2: Stability Analysis
Computes eigenvalues of \( A \) and determines asymptotic stability.

### Part 3: BIBO Stability
Uses MATLAB’s `isstable()` to evaluate input–output stability.

### Part 4: Transfer Function
Extracts the scalar transfer function \( G_{11}(s) \) from input \( u_1 \) to output \( y_1 \).

### Part 5: Open-Loop Step Response
Simulates output behavior under a step input applied to all input channels.

### Part 6: Controllability and Observability
Verifies rank conditions using `ctrb()` and `obsv()`.

### Part 7: LQR Controller Design
Computes optimal feedback gain \(K\), evaluates closed-loop eigenvalues, and plots step responses.

### Part 8: Full-State Estimator
Designs a Luenberger observer using pole placement.

### Part 9: Output-Feedback Control
Constructs the observer–controller augmented system and simulates closed-loop behavior.

### Part 10: Zero-Input Response
Simulates response due to nonzero initial conditions to evaluate disturbance rejection.

### Part 11: Reference Tracking
Computes feedforward gain \(N\) and demonstrates accurate step tracking for the first output.

All generated figures are saved into the `figures/` directory for use in the report.

---

## How to Run

1. Clone the repository:
```bash
git clone https://github.com/rmranjitkarNULL/helicopter-lqr-estimator-control.git
```

3. Open MATLAB and navigate to the src/ folder

4. Run the main script:

5. The script will:
- Simulate all system responses,
- Print analysis results to the console,
- Print figues of responses

## Requirements
- MATLAB R2020a or newer
- Control System Toolbox
- Signal Processing Toolbox (optional)

# Summary of Results
- The helicopter system is open-loop unstable.
- It is fully controllable and observable.
- The LQR controller stabilizes the system and provides well-damped dynamics.
- The full-state estimator reconstructs all states accurately.
- The observer-based output-feedback controller maintains stability and performs comparably to full-state feedback.
- The reference tracking controller ensures zero steady-state error for the first output.

Complete discussion and analysis are documented in the LaTeX report.

# Author

Ryan Ranjitkar
Worcester Polytechnic Institute
ECE 504 – Fall 2025
