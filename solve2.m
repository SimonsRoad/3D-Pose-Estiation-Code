% given I0 J0 u, use complex number to calculate two solutions
function [I1 J1 I2 J2] = solve2(I0, J0, u)

C2 = J0*J0'-I0*I0'-2i*I0*J0';
C = sqrt(C2);
rho = abs(C);
theta = angle(C);

I1 = I0 + u * rho * cos(theta);
J1 = J0 + u * rho * sin(theta);

I2 = I0 - u * rho * cos(theta);
J2 = J0 - u * rho * sin(theta);

end