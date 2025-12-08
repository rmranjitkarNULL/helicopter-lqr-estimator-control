function L = designEstimator(A, C, closedLoopPoles)

    disp("Part 8 – Estimator Design");

    desiredPoles = 3 * closedLoopPoles;
    L = place(A', C', desiredPoles).';

    disp("Observer Gain L:");
    disp(L);

end
