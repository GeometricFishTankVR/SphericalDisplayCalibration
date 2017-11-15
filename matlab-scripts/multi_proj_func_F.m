function [ err, J ] = multi_proj_func_F( p )
% Two projector example of p:
% p = [ fx_p1, fy_p1, cx_p1, cy_p1, //1-4 proj1 intrinsic
%       om11, om12, om13, //5-7 proj1 rotation
%       t11, t12, t13, //8-10 proj1 translation
%       fx_p2, fy_2, cx_p2, cy_p2, //11-14 proj2 intrinsic
%       om21, om22, om23, //15-17 proj2 rotation
%       t21, t22, t23, //18-20 proj2 translation
%       ... //etc proj3...
%       a, b, c, r, ///?1-?4 sphere pose
%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //?5-?3 cam
% p has to be a column vector

num_proj = (length(p) - 13)/10;

err = null(1);
J = null(1);

for idx = 1:num_proj
    vec_p = [p( idx*10-9 : idx*10 ); 
                      p( num_proj*10+1 : num_proj*10+13 )];
    [err_p, A, B, C] = func_F(vec_p, idx);   
    len = length(A);
    J_row = zeros(len, num_proj*10);
    range = (idx*10-9) : (idx*10);
    J_row(:, range) = A;
    J = [J; J_row B C];
    err = [err;err_p];  
end

end

