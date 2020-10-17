function Q = planarIK(pnt,m)
if nargin == 1
    m = -1;
end
x = pnt(1);
y = pnt(2);
z = pnt(3);

l1 = 1; l2 = 1; l3 = 1;
 
q1 =pi/2 - atan2(y,x);
q3 = acos((x^2+(z-l1)^2-l3^2-l2^2)/(2*l3*l2));
q2 = -m*acos((l3^2-l2^2-x^2-(z-l1)^2)/(-2*l2*sqrt(x^2+(z-l1)^2))) + atan2(z-l1, y);
Q = [q1,q2,q3];
end

