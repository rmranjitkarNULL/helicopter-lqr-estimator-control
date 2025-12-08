function plotClosedLoopStep(A_aug, B_aug, C_aug, D_aug, T)

    disp("Part 9 – Closed-loop step response (output feedback)");

    sys = ss(A_aug, B_aug, C_aug, D_aug);

    figure('Name','Part 9: Output Feedback Step Response');
    step(sys(:,1), T);
    title("Closed-Loop Output Response with Output Feedback");
    grid on;

end
