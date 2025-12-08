function [eigVals, asymStable, biboStable] = checkStability(A, sys)

    disp("Part 2 & 3 – Stability Analysis");

    eigVals = eig(A);
    disp("Eigenvalues:");
    disp(eigVals);

    asymStable = all(real(eigVals) < 0);
    biboStable = isstable(sys);

    if asymStable
        disp("Asymptotically stable.");
    else
        disp("NOT asymptotically stable.");
    end

    if biboStable
        disp("BIBO stable.");
    else
        disp("BIBO UNSTABLE.");
    end

end
