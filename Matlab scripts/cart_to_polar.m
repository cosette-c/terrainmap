function [r, theta] = cart_to_polar(x, y) 
r = sqrt(x.^2+y.^2) ; 
if x = 0
    theta = pi/2 ;
else
    theta = atan(y./x);
end
end