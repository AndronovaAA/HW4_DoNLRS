function pnt = robotFK(Q)
%  ROBOTFK Forward Kinematics
q1 = Q(1,:);
q2 = Q(2,:);
q3 = Q(3,:);
l1 = 1;
l2 = 1;
l3 = 1;
for i = 1:length(q1)
    x(i) = sin(q1(i))*(l2*cos(q2(i))+l3*cos(q2(i)+q3(i)));
    y(i) = -cos(q1(i))*(l2*cos(q2(i))+l3*cos(q2(i)+q3(i)));
    z(i) = l1+sin(q2(i))*l2+sin(q3(i)+q2(i))*l3;
end
pnt = [x;y;z];
end