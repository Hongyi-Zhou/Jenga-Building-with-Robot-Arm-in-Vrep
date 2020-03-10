function [t1,t2,t3] = RtoA(R)
    t2 = asind(R(1,3));
    %t22 = 180-t21
    t3 = -atan2d(R(1,2)/cosd(t2),R(1,1)/cosd(t2));
    %t32 = -atan2d(T(1,2)/cosd(t22),T(1,1)/cosd(t22))
    t1 = -atan2d(R(2,3)/cosd(t2),R(3,3)/cosd(t2));
end