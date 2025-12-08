function [A, B, C, D, sys] = loadHelicopterSystem()
    [A, B, C, D] = helecopterModel();
    sys = ss(A, B, C, D);
end
