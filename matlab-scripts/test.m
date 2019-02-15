syms w11 w12 w13 w21 w22 w23 w31 w32 w33
syms a11 a12 a13 a21 a22 a23 a31 a32 a33
syms a1 a2 a3
syms p1 p2 p3

PA = [a11 a12 a13; a21 a22 a23; a31 a32 a33]
Pa = [a1;a2;a3]
wc_d = [w11 w12 w13; w21 w22 w23; w31 w32 w33]
p = [p1;p2;p3]

w_p_d = (PA - Pa * p.') * wc_d * (PA - Pa * p.').'

constrain1 = w_p_d(1,3) - 512 * w_p_d(3,3);
constrain2 = w_p_d(2,3) - 768 * w_p_d(3,3);
constrain3 = w_p_d(1,2) - 512 * 768 * w_p_d(3,3);

%matlabFunction(constrain1,'File','Constrain1','Vars',[p1,p2,p3,w11,w12,w13,w21,w22,w23,w31,w32,w33,a11,a12,a13,a21,a22,a23,a31,a32,a33,a1,a2,a3])
%matlabFunction(constrain1,'File','Constrain1','Vars',[p1,p2,p3,w11,w12,w13,w21,w22,w23,w31,w32,w33,a11,a12,a13,a21,a22,a23,a31,a32,a33,a1,a2,a3])
%matlabFunction(constrain1,'File','Constrain1','Vars',[p1,p2,p3,w11,w12,w13,w21,w22,w23,w31,w32,w33,a11,a12,a13,a21,a22,a23,a31,a32,a33,a1,a2,a3])