function [theta, count] = inverse(T_1in0, M, S)
disp(T_1in0);
n = size(S); % the number of joints
n = n(2);
xpos = T_1in0(1,4);
ypos = T_1in0(2,4);
zpos = T_1in0(3,4);
if (xpos>-0.3 && ypos >-0.2)%tower
    if (zpos<0.94)
        theta =[2.85593529082406;0.720126733884008;1.71893638191425;-0.868266789002227;-1.5707963267949;-0.285657362765744];
    else
        theta =[2.80781747713305;0.115108760984024;1.32303129222757;0.132656273583307;-1.5707963267949;-0.333775176456746];
    end
elseif (xpos>=-0.5 && ypos>=0) %lowerleft
    if (zpos<0.85)
        theta = [-2;0.720126733884008;1.71893638191425;-0.868266789002227;-1.5707963267949;-0.285657362765744];
    else
        theta = [-2.17848630094886;0.774910534049056;0.794458630270057;0.00142716247578339;-1.5707963267949;1.12309482302106];
    end
elseif ((xpos>=-0.65) && ypos<0)   %upper left
    if (zpos<0.85)
        theta = [1.2645179934599;0.862871170450163;1.38713931786071;-0.679214161515974;-1.5707963267949;-3.12789404616453];
    else
        theta = [1.26311862089365;0.611786041120864;1.12563721760125;-0.166626931927218;-1.5707963267949;-3.12929341873078];
    end
elseif (xpos<-0.5 && ypos>0.25) %lower right
    if (zpos<0.85)
        theta = [-1.48732946383578;0.86453792988792;1.38392934935745;-0.677670952450471;-1.5707963267949;1.27610842971654];
    else
        theta = [-1.48733190043575;0.6102968748653;1.07930581183155;-0.118806359901957;-1.5707963267949;1.27610599311657];
    end
elseif (xpos<-0.5 && (ypos>=-0.25 && ypos<=0.25)) %middle right
    if (zpos<0.85)
        theta = [-0.371190782946618;0.639502984443398;1.86254485074305;-0.931251508391631;-1.57079632679475;-2.48013020700106];
    else
        theta = [-0.37120594345825;0.282626333254009;1.57158266894936;-0.283412675408474;-1.5707963267949;-2.48014536751269];
    end
else 
    if (zpos<0.85) %upper right
        theta = [0.619331819009485;0.89484735526518;1.31991786942069;-0.64396889789097;-1.5707963267949;-1.83867352202285];
    else
        theta = [0.619331531023194;0.653900624533801;1.00616439936782;-0.0892686971067239;-1.5707963267949;-1.83867381000914];
    end
end

tol = 0.001; % the tolerence between the solution and the correct answer
count = 0; % the count of loops
T_2in0 = T_1in0;
Tt = T_1in0;
mu = 0.1; % greek charactor mu

while 1
    count = count+1;
    if count > 2000
        break
    end
    J = jacobian(S,theta);
    T_1in0 = forward(S,theta,M);
    T = T_2in0/T_1in0;
    V = ascrew(logm(T));
    error = norm(V);
    if error < tol
        break
    else
        if n == 6
            dtheta = inv(J)*V;
        elseif n < 6
            dtheta = (J.'*J)\(J.'*V);
        elseif n > 6
            dtheta = (J.'*J + mu*eye(n)) \ (J.'*V);
        end
        theta = theta+dtheta;
    end
end

% make theta(i) < pi && theta(i) > -pi

for i = 1:n
    flag = 0;
    while flag == 0
        if theta(i) > pi
            theta(i) = theta(i)-2*pi;
        elseif theta(i) < -pi
            theta(i) = theta(i)+2*pi;
        else
            flag = 1;
        end
    end
end

if count == 2001 
    disp(['Inverse Kinematics Calculation Failed']);
    disp(Tt);
    return
end 

end