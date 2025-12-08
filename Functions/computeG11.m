function G11 = computeG11(sys)
    G = tf(sys);
    G11 = G(1,1);
end
