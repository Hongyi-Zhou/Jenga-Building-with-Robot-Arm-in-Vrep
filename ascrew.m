function [ascrew] = ascrew(V)
w = V(1:3,1:3);
v = V(1:3, 4);
ascrew= [askew(w); v];
end