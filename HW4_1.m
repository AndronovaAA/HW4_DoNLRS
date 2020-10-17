clear; clc;

syms l1 real;
syms l2 real;
syms l3 real;

syms theta1 real;
syms theta2 real;
syms theta3 real;


%% Jacobian numerical
T_0 = simplify(Rz(theta1)*Tz(l1)*Rx(theta2)*Ty(l2)*Rx(theta3)*Ty(l3));
R0 = simplify(T_0(1:3, 1:3));

J1r = Rzd(theta1)*Tz(l1)*Rx(theta2)*Ty(l2)*Rx(theta3)*Ty(l3)*...
    [R0^-1 zeros(3,1); 0 0 0 1];
J1 = [J1r(1,4) J1r(2,4) J1r(3,4) J1r(3,2) J1r(1,3) J1r(2,1)]';

J2r = Rz(theta1)*Tz(l1)*Rxd(theta2)*Ty(l2)*Rx(theta3)*Ty(l3)*...
    [R0^-1 zeros(3,1); 0 0 0 1];
J2 = [J2r(1,4) J2r(2,4) J2r(3,4) J2r(3,2) J2r(1,3) J2r(2,1)]';

J3r = Rz(theta1)*Tz(l1)*Rx(theta2)*Ty(l2)*Rxd(theta3)*Ty(l3)*...
    [R0^-1 zeros(3,1); 0 0 0 1];
J3 = [J3r(1,4) J3r(2,4) J3r(3,4) J3r(3,2) J3r(1,3) J3r(2,1)]';

J_N = [simplify(J1), simplify(J2), simplify(J3)]

