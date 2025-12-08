function [ctrlOK, obsOK] = checkControllabilityObservability(A, B, C)

    disp("Part 6 – Controllability & Observability");

    ctrlOK = (rank(ctrb(A,B)) == size(A,1));
    obsOK  = (rank(obsv(A,C)) == size(A,1));

    if ctrlOK
        disp("System is CONTROLLABLE.");
    else
        disp("System is NOT controllable.");
    end

    if obsOK
        disp("System is OBSERVABLE.");
    else
        disp("System is NOT observable.");
    end

end
