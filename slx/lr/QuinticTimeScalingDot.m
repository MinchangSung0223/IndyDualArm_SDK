function s = QuinticTimeScalingDot(Tf, t)
s = 3*10 *(1/ Tf)^3* t ^ 2 - 4*15 * (1/ Tf) ^ 4 *t^3 + 5*6 * (1 / Tf) ^ 5*t^4;
end