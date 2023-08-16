function [N,T,L,wref,wv,wa,ws,uH,uL] = Initialize(Nominal_u,Nominal_x)
%% We can initialize differently based on the problem we want to solve
N = length(Nominal_x);
T = 0.2;
L = 1.5;
wref = 0.09;
wv = 1;
wa = 1;
ws = 10;
uH = 2;
uL = -2;
end