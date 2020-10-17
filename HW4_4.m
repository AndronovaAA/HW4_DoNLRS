clear; clc;

p1 = [1, 0, 1];
p2 = [sqrt(2)/2, sqrt(2)/2, 1.2];

q_00 = planarIK(p1,1);
q_f = planarIK(p2,-1);

q10 = q_00(1); q1f = q_f(1);
q20 = q_00(2); q2f = q_f(2);
q30 = q_00(3); q3f = q_f(3);

v = 1;
a = 10;
v10 = 0; v20 = 0; v30 = 0; % initial velocity for joints
delta_t = 0.01;
n = 0;

while (floor(delta_t*10^n)~=delta_t*10^n)
    n=n+1;
end
E = 1*10^-n;
t1a = v/a;
if rem(t1a,delta_t)~=0
    t1a_new = round(t1a,n)+E;
else
    t1a_new = round(t1a,n);
end

t1f = (q1f-q10)/v + t1a_new;
if rem(t1f,delta_t)~=0
    t1f_new = round(t1f,n)+E;
else
    t1f_new = round(t1f,n);
end

t2a = v/a;
if rem(t2a,delta_t)~=0
    t2a_new = round(t2a,n)+E;
else
    t2a_new = round(t2a,n);
end

t2f = (q2f-q20)/v + t2a_new;
if rem(t2f,delta_t)~=0
    t2f_new = round(t2f,n)+E;
else
    t2f_new = round(t2f,n);
end

t3a = v/a;
if rem(t3a,delta_t)~=0
    t3a_new = round(t3a,n)+E;
else
    t3a_new = round(t3a,n);
end

t3f = (q3f-q30)/v + t3a_new;
if rem(t3f,delta_t)~=0
    t3f_new = round(t3f,n)+E;
else
    t3f_new = round(t3f,n);
end

if t2f_new > t1f_new && t2f_new > t3f_new 
    tf_new = t2f_new;
    ta_new = t2a_new;
elseif t1f_new > t2f_new && t1f_new > t3f_new 
    tf_new = t1f_new;
    ta_new = t1a_new;
else 
    tf_new = t3f_new;
    ta_new = t3a_new;
end

N = 100;
tspan = linspace(ta_new,tf_new,N);

x = ((p2(1)-p1(1))/tf_new).*tspan+p1(1);
y = ((p2(2)-p1(2))/tf_new).*tspan+p1(2);
z = ((p2(3)-p1(3))/tf_new).*tspan+p1(3);

m=1;



for i=1:N
    waypnts = [x(i), y(i), z(i)];
    jointConfig(i,:) = planarIK(waypnts(1,:),m);
end

for i = 1:N-1
    t1 = tspan(i); t2 = tspan(i+1);
    
    q1_f = jointConfig(i+1,1);
    q1_0 = jointConfig(i,1);
    q2_f = jointConfig(i+1,2);
    q2_0 = jointConfig(i,2);
    q3_f = jointConfig(i+1,3);
    q3_0 = jointConfig(i,3);
    
    v1_new = ((q1_f-q1_0)/(t2-t1));
    a1_new = v1_new/t1;

    v2_new = ((q2_f-q2_0)/(t2-t1));
    a2_new = v2_new/t1;

    v3_new = ((q3_f-q3_0)/(t2-t1));
    a3_new = v3_new/t1;
    
    vel = [v1_new; v2_new; v3_new];
    acc = [a1_new; a2_new; a3_new];
    
    if i == 1 || i == N
        jointVel(i,:) = [0 0 0];
    else
        J = computeJac(jointConfig(i,:));
        jointVel(i,:) =  (J\vel)';
        jointAcc(i,:) = (J\acc)';
    end 
end

n = 20;
t_n =[];
Q = [];
Qd = [];
Qdd = [];
for i=1:N-2
    t0 = tspan(i); tf = tspan(i+1);
    A = [1 t0 t0^2 t0^3 t0^4 t0^5
    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
    0 0 2 6*t0 12*t0^2 20*t0^3
    1 tf tf^2 tf^3 tf^4 tf^5
    0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
    0 0 2 6*tf 12*tf^2 20*tf^3];
    for j = 1:3
        q1 = jointConfig(i,j); q2 = jointConfig(i+1,j);
        v1 = jointVel(i,j); v2 = jointVel(i+1,j);
        a1 = jointAcc(i,j); a2 = jointAcc(i+1,j);
        c = [q1;v1;a1;q2;v2;a2];
        b = A\c;
        t = t0:0.1:tf;
        q_n(j,:) = b(1)+b(2).*t+b(3).*t.^2+b(4).*t.^3+b(5).*t.^4+b(6).*t.^5;
        v_n(j,:) = b(2)+2*b(3).*t+3*b(4).*t.^2+4*b(5).*t.^3+5*b(6).*t.^4;
        acc_n(j,:) = 2*b(3)+6*b(4).*t+12*b(5).*t.^2+20*b(6).*t.^3;
    end
    t_n = [t_n tf];
    Q = [Q q_n];
    Qd = [Qd v_n];
    Qdd = [Qdd acc_n];
end 

%plot(t_n,Q(1,:),'g','linewidth',2)

figure
plot(t_n,Q(1,:),'g-')
hold on
grid on
plot(t_n,Q(2,:),'b-')
plot(t_n,Q(3,:),'r-')
title('position vs time')
legend('joint1','joint2','joint3')

figure
plot(t_n,Qd(1,:),'g-')
hold on
grid on
plot(t_n,Qd(2,:),'b-')
plot(t_n,Qd(3,:),'r-')
title('velocity vs time')
legend('joint1','joint2','joint3')

figure
plot(t_n,Qdd(1,:),'g-')
hold on
grid on
plot(t_n,Qdd(2,:),'b-')
plot(t_n,Qdd(3,:),'r-')
title('acceleration vs time')
legend('joint1','joint2','joint3')


cartTraj = robotFK(Q);

%% Figures

l1 = 1; l2 = 1; l3 = 1;
q1 = jointConfig(1,1);
q2 = jointConfig(1,2);
q3 = jointConfig(1,3);

figure
plot3(x,y,z)
hold on
plot3(cartTraj(1,:),-cartTraj(2,:),cartTraj(3,:),'c--')
line([-3 3],[0 0],[0,0],'Color','black','LineStyle','--')
line([0 0],[-3 3],[0 0],'Color','black','LineStyle','--')
line([0 0],[0 0],[-3 3],'Color','black','LineStyle','--')
hold on
line([0 0],[0 0], [0 l1], 'linewidth', 3,'Color','black')
line([0 sin(q1)*l2*cos(q2)],[0 -l2*cos(q2)*cos(q1)], [l1 l1+l2*sin(q2)], 'linewidth', 3,'Color','black')
line([sin(q1)*l2*cos(q2) sin(q1)*(l2*cos(q2)+l3*cos(q2+q3))],[-l2*cos(q2)*cos(q1) -cos(q1)*(l2*cos(q2)*l3*cos(q2+q3))], [l1+l2*sin(q2) l1+sin(q2)*l2+sin(q3+q2)*l3], 'linewidth', 3,'Color','black')
