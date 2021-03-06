function J = computeJac(Q)
l1 = 1;
l2 = 1;
l3 = 1;
theta1 = Q(1,1);
theta2 = Q(1,2);
theta3 = Q(1,3);
J = [[-cos(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2)), sin(theta1)*(l3*sin(theta2 + theta3) + l2*sin(theta2)), l3*sin(theta2 + theta3)*sin(theta1)];...
     [-sin(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2)), -cos(theta1)*(l3*sin(theta2 + theta3) + l2*sin(theta2)), -l3*sin(theta2 + theta3)*cos(theta1)];...
     [0,   l3*cos(theta2 + theta3) + l2*cos(theta2),   l3*cos(theta2 + theta3)]];
%      [0,  cos(theta1), cos(theta1)],...
%      [0, sin(theta1), sin(theta1)],...
%      [1, 0, 0]];