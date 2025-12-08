function simulateZeroInput(A_aug, C_aug, x0, T)

    disp("Part 10 – Zero-input initial condition response");

    n = size(A_aug,1);
    B_zero = zeros(n, size(C_aug,1));
    D_zero = zeros(size(C_aug,1));

    sys_zero = ss(A_aug, B_zero, C_aug, D_zero);

    xhat0 = zeros(size(x0));
    X0 = [x0; xhat0];

    figure('Name','Part 10: Zero-Input Response');
    initial(sys_zero, X0, T);
    title("Zero-Input Response of Closed-Loop System");
    grid on;

end
