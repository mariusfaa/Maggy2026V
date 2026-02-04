function [A,B] = casadiFiniteDifferenceLinearization(f,xLp,uLp,delta)
nStates     = length(xLp);
nInputs     = length(uLp);

A = zeros(nStates);
B = zeros(nStates,nInputs);

for i = 1:nStates
    A(:,i) = (f(xLp+delta*(i==1:nStates)',uLp)-f(xLp-delta*(i==1:nStates)',uLp))/(2*delta);
end
A = round(A,5);

for i = 1:nInputs
    B(:,i) = (f(xLp,uLp+delta*(i==1:nInputs)')-f(xLp,uLp-delta*(i==1:nInputs)'))/(2*delta);
end
B = round(B,5);