function checkStability(lambda)

    tol = 1e-6;  % tolerance for floating point comparisons
    realParts = real(lambda);

    if all(realParts < -tol)
        disp("System is Asymptotically Stable");

    elseif any(realParts > tol)
        disp("System is Unstable");

    else
        disp("System is Marginally Stable");
    end

end
