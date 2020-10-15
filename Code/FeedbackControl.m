function [Vd, V, Je, u_theta_dot, Xerr, Xerr_integral] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t, thetaList, Xerr_integral)

%% Arm properties.
Blist = [0 0 1 0 0.033 0; 0 -1 0 -0.5076 0 0; 0 -1 0 -0.3526 0 0; 0 -1 0 -0.2176 0 0; 0 0 1 0 0 0]';
l = 0.47/2;
w = 0.30/2;
r = 0.0475;
F = (r/4) * [-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w); 1 1 1 1; -1 1 -1 1];
sizee = size(F);
m = sizee(2);
zeross = zeros(1, m);
F6 = [zeross; zeross; F; zeross];

Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
T0e = FKinBody(M0e, Blist, thetaList);
Tbe = Tb0 * T0e;
Teb = TransInv(Tbe);
Jbase = Adjoint(Teb) * F6;
Jarm = JacobianBody(Blist, thetaList);
Je = [Jbase, Jarm];
psInv = pinv(Je, 1e-2);

Xerr_bracket = MatrixLog6(TransInv(X) * Xd);
Xerr = se3ToVec(Xerr_bracket);
Xerr_integral = Xerr_integral + (delta_t * Xerr);
Vd_bracket = (1/delta_t) * (MatrixLog6(inv(Xd) * Xd_next));
Vd = se3ToVec(Vd_bracket);

Adj = Adjoint(inv(X) * Xd);
Feedforward = Adj * Vd;
V = Feedforward + (Kp * Xerr) + Ki * Xerr_integral;
%V = round(V, 3);

u_theta_dot = psInv * V;
%u_theta_dot = round(u_theta_dot, 1);

%u_theta_dot(1) = temp(5);
%u_theta_dot(2) = temp(6);
%u_theta_dot(3) = temp(7);
%u_theta_dot(4) = temp(8);
%u_theta_dot(5) = temp(9);
%u_theta_dot(6) = temp(1);
%u_theta_dot(7) = temp(2);
%u_theta_dot(8) = temp(3);
%u_theta_dot(9) = temp(4);

end