function plotStepAllInputs(A, B, C, D, T)
    disp("Part 5 – Step response (all inputs)");

    u = ones(size(B,2),1);
    sys_sum = ss(A, B*u, C, D*u);

    figure('Name','Part 5: Step Response');
    stepplot(sys_sum, T);
    title("Output Response to Unit Step in All Inputs");
    grid on;
end
