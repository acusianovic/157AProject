function F = RaoFx(X)
%% Declare Global Variables
global Xe Ye alpha_n alpha_e

%{
The F vector will represent your 4 equations you should have derived in
question c of part IV. For example, F(1) is equal to the 1st equation and
F(2) is equal to the 2nd equation, etc. All your terms in each equation
should be on one side, with the entire equation being equal to zero.

A vector x will represent your unknowns. This will have the following
format:

x = [P ; Q; S; T] therefore, x(1)=P, x(2)=Q, etc.

Example:
If your first equation was something like: P + 2Q + 4T^2 + 1 = 0
This would be written below as: F(1) = x(1) + 2*x(2) + 4*x(4)^2 + 1

Fill in F(1) - F(4) with the equations you derived:
%}

F(1) = X(2)+X(4)^0.5;
F(2) = X(1)*Xe+X(2)+(X(3)*Xe+X(4))^0.5-Ye;
F(3) = tand(alpha_n)-X(1)-X(3)/2/X(4)^0.5;
F(4) = tand(alpha_e)-X(1)-X(3)/2/(X(3)*Xe+X(4))^0.5;
