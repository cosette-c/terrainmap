function [r_error, theta_error] = noise(matrix, sigma_angle, sigma_distance)

for i = 1:size(matrix, 1)
    r_error(i) = normrand(matrix(i, 1), sigma_distance)  ;
    theta_error(i) = normrand(matrix(i, 2), sigma_angle) ;
end
end