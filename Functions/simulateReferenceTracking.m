function simulateReferenceTracking(A, B, C, D, K, L, T)

    disp("Part 11 – Reference Tracking Design");

    Acl = A - B*K;
    b1 = B(:,1);
    c1 = C(1,:);

    N = 1 / (c1 * (-Acl \ b1));

    [A_aug, ~, ~, ~] = buildAugmentedOutputFeedbackSystem(A, B, C, D, K, L);

    B_ref = [b1*N;
             zeros(size(b1))];

    C_y1 = [c1, zeros(1, size(A,1))];
    D_y1 = 0;

    sys_track = ss(A_aug, B_ref, C_y1, D_y1);

    figure('Name','Part 11: Reference Tracking');
    step(sys_track, T);
    title("Tracking Response — y1(t) Tracking Unit Step Reference");
    grid on;

end
