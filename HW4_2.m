clear; clc;

t0 = 0; tf = 2;
q0 = [0, 0, 0]; 
qf = [2, 3, 4]; 
v0 = 0; vf = 0;
acc0 = 0; accf = 0;
N = 100;
tspan = linspace(0,tf,N);

A = [1 t0 t0^2 t0^3 t0^4 t0^5
0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
0 0 2 6*t0 12*t0^2 20*t0^3
1 tf tf^2 tf^3 tf^4 tf^5
0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
0 0 2 6*tf 12*tf^2 20*tf^3];

for j = 1:3
    q1 = q0(j); q2 = qf(j);
    v1 = v0; v2 = vf;
    a1 = acc0; a2 = accf;
    c = [q1;v1;a1;q2;v2;a2];
    b = A\c;
    t = t0:0.1:tf;
    q_n(j,:) = b(1)+b(2).*t+b(3).*t.^2+b(4).*t.^3+b(5).*t.^4+b(6).*t.^5;
    v_n(j,:) = b(2)+2*b(3).*t+3*b(4).*t.^2+4*b(5).*t.^3+5*b(6).*t.^4;
    acc_n(j,:) = 2*b(3)+6*b(4).*t+12*b(5).*t.^2+20*b(6).*t.^3;
end   

figure
plot(t,q_n(1,:),'g-')
hold on
grid on
plot(t,q_n(2,:),'b-')
plot(t,q_n(3,:),'r-')
title('position vs time')
legend('joint1','joint2','joint3')

figure
plot(t,v_n(1,:),'g-')
hold on
grid on
plot(t,v_n(2,:),'b-')
plot(t,v_n(3,:),'r-')
title('velocity vs time')
legend('joint1','joint2','joint3')

figure
plot(t,acc_n(1,:),'g-')
hold on
grid on
plot(t,acc_n(2,:),'b-')
plot(t,acc_n(3,:),'r-')
title('acceleration vs time')
legend('joint1','joint2','joint3')